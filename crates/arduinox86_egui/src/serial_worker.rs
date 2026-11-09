/*
    ArduinoX86 Copyright 2022-2025 Daniel Balsom
    https://github.com/dbalsom/arduinoX86

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
use std::{
    io::Cursor,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};

use anyhow::{anyhow, Context, Result};
use arduinox86_client::{
    CpuClient,
    CpuClientError,
    CpuPin,
    ProgramState,
    RegisterSetType,
    RemoteCpuRegisters,
    ServerCpuType,
    ServerCycleState,
    ServerFlags,
    ServerStatus,
};
use crossbeam_channel::{Receiver, Sender, TryRecvError};

use crate::{client::ClientContext, memory_upload::MemoryUpload, serial_manager::SerialManager};
use crate::disassembly::{read_disassembly, DisassemblyReadout, DisassemblyRequest};
use crate::enums::ExecutionMode;
use arduinox86_client::trap::{InstructionAddress, InstructionState, TrapStatus};

pub enum SerialCommand {
    ListPorts,
    Connect(String),
    Disconnect,
    Poll,
    Load { regs: RemoteCpuRegisters, run: bool, uploads: Vec<MemoryUpload> },
    Step,
    StepInstruction,
    ResetCpu,
    ContinueTrap,
    SetTrapEndpoint { cs: u16, ip: u16, enabled: bool },
    SetExecutionMode(ExecutionMode),
    SetFlag { flag: u32, enabled: bool },
    WritePin { pin: CpuPin, value: bool },
    WriteDataBus(u16),
    ReadMemory { address: u32, size: u32 },
    ReadDisassembly(DisassemblyRequest),
    Upload { upload: MemoryUpload, regs: RemoteCpuRegisters },
    EraseMemory,
    ClearCycleLog,
}

impl SerialCommand {
    pub fn label(&self) -> &'static str {
        match self {
            Self::ListPorts => "Finding serial ports",
            Self::Connect(_) => "Connecting",
            Self::Disconnect => "Disconnecting",
            Self::Poll => "Reading CPU status",
            Self::Load { run: true, .. } => "Starting program",
            Self::Load { .. } => "Loading registers",
            Self::Step => "Stepping CPU",
            Self::StepInstruction => "Stepping guest instruction",
            Self::ResetCpu => "Resetting CPU",
            Self::ContinueTrap => "Running to trap endpoint",
            Self::SetTrapEndpoint { .. } => "Setting trap endpoint",
            Self::SetExecutionMode(_) => "Changing execution mode",
            Self::SetFlag { .. } => "Updating server options",
            Self::WritePin { .. } => "Updating CPU input",
            Self::WriteDataBus(_) => "Writing data bus",
            Self::ReadMemory { .. } => "Reading memory",
            Self::ReadDisassembly(_) => "Reading disassembly",
            Self::Upload { .. } => "Uploading binary",
            Self::EraseMemory => "Erasing memory",
            Self::ClearCycleLog => "Clearing cycle log",
        }
    }
}

pub struct PinReadout {
    pub pin:   CpuPin,
    pub value: Result<bool, CpuClientError>,
}

#[derive(Default)]
pub struct SerialUpdate {
    pub disassembly: Option<DisassemblyReadout>,
    pub loaded_regs: Option<RemoteCpuRegisters>,
    pub memory_changed: bool,
    pub instruction: Option<arduinox86_client::trap::InstructionState>,
    pub trap_endpoint: Option<Option<InstructionAddress>>,
    pub status: Option<ServerStatus>,
    pub flags: Option<u32>,
    pub pins: Vec<PinReadout>,
    pub cycle: Option<ServerCycleState>,
    pub cycles: Option<Vec<ServerCycleState>>,
    pub registers: Option<(RemoteCpuRegisters, RemoteCpuRegisters)>,
    pub memory: Option<(u32, Vec<u8>)>,
    pub notice: Option<String>,
}

pub enum SerialOutcome {
    Ports(Vec<serialport::SerialPortInfo>),
    Connected(ClientContext, SerialUpdate),
    Disconnected,
    Update(SerialUpdate),
}

pub struct SerialReply {
    pub operation: &'static str,
    pub result:    Result<SerialOutcome, String>,
}

/// Only messages cross this boundary. CpuClient (including its Rc/RefCell)
/// is created, used, and dropped exclusively on the serial thread.
pub struct SerialWorker {
    requests: Sender<SerialCommand>,
    replies: Receiver<SerialReply>,
    pending: Option<&'static str>,
    pending_since: Instant,
    pending_background: bool,
    queued_action: Option<SerialCommand>,
    disconnecting: bool,
    stopping: Arc<AtomicBool>,
}

impl SerialWorker {
    pub fn new(ctx: egui::Context) -> Self {
        Self::spawn(ctx, run_worker)
    }

    pub(super) fn spawn(
        ctx: egui::Context,
        run: impl FnOnce(Receiver<SerialCommand>, Sender<SerialReply>, egui::Context, Arc<AtomicBool>) + Send + 'static,
    ) -> Self {
        let (requests, incoming) = crossbeam_channel::bounded(1);
        let (outgoing, replies) = crossbeam_channel::unbounded();
        let stopping = Arc::new(AtomicBool::new(false));
        let worker_stopping = stopping.clone();
        thread::Builder::new()
            .name("arduinox86-serial".into())
            .spawn(move || run(incoming, outgoing, ctx, worker_stopping))
            .expect("Failed to start serial worker");
        Self {
            requests,
            replies,
            pending: None,
            pending_since: Instant::now(),
            pending_background: false,
            queued_action: None,
            disconnecting: false,
            stopping,
        }
    }

    pub fn busy(&self) -> bool {
        self.pending.is_some() || self.queued_action.is_some() || self.disconnecting
    }

    /// Background reads keep the controls visually stable and can accept one
    /// user action to run after their complete reply has been consumed.
    pub fn controls_busy(&self) -> bool {
        self.disconnecting || self.queued_action.is_some() || (self.pending.is_some() && !self.pending_background)
    }

    pub fn activity(&self) -> Option<&'static str> {
        if self.disconnecting {
            Some("Disconnecting (waiting for current operation)")
        }
        else {
            self.pending
                .filter(|_| !self.pending_background && self.pending_since.elapsed() >= Duration::from_millis(250))
        }
    }

    pub fn submit(&mut self, command: SerialCommand) -> Result<()> {
        if matches!(command, SerialCommand::Poll) {
            return self.submit_background(command);
        }
        if self.controls_busy() {
            return Err(anyhow!("The serial connection is busy."));
        }
        if self.pending.is_some() {
            self.queued_action = Some(command);
            return Ok(());
        }
        self.send(command, false)
    }

    pub fn submit_background(&mut self, command: SerialCommand) -> Result<()> {
        if self.busy() {
            return Err(anyhow!("The serial connection is busy."));
        }
        self.send(command, true)
    }

    fn send(&mut self, command: SerialCommand, background: bool) -> Result<()> {
        let label = command.label();
        self.requests
            .try_send(command)
            .map_err(|_| anyhow!("The serial worker is unavailable."))?;
        self.pending = Some(label);
        self.pending_since = Instant::now();
        self.pending_background = background;
        Ok(())
    }

    pub fn disconnect(&mut self) -> Result<()> {
        if self.disconnecting {
            return Ok(());
        }
        self.queued_action = None;
        if self.pending.is_none() {
            self.send(SerialCommand::Disconnect, false)?;
        }
        self.disconnecting = true;
        Ok(())
    }

    pub fn try_recv(&mut self) -> Option<SerialReply> {
        match self.replies.try_recv() {
            Ok(reply) => {
                self.pending = None;
                if self.disconnecting && !matches!(reply.result, Ok(SerialOutcome::Disconnected)) {
                    // Discard results belonging to the closing connection. Close
                    // its port before allowing another Connect to be submitted.
                    if let Err(error) = self.send(SerialCommand::Disconnect, false) {
                        self.disconnecting = false;
                        return Some(SerialReply {
                            operation: "Disconnecting",
                            result:    Err(error.to_string()),
                        });
                    }
                    return None;
                }
                self.disconnecting = false;
                if let Some(command) = self.queued_action.take() {
                    let operation = command.label();
                    if let Err(error) = reply.result {
                        return Some(SerialReply {
                            operation,
                            result: Err(format!(
                                "Action not started because the background read failed: {error}"
                            )),
                        });
                    }
                    if let Err(error) = self.send(command, false) {
                        return Some(SerialReply {
                            operation,
                            result: Err(error.to_string()),
                        });
                    }
                    // This read predates the queued action. In particular, don't
                    // reopen an old final-register window after a new load click.
                    return None;
                }
                Some(reply)
            }
            Err(TryRecvError::Empty) => None,
            Err(TryRecvError::Disconnected) => {
                let operation = self.pending.take()?;
                self.queued_action = None;
                self.disconnecting = false;
                Some(SerialReply {
                    operation,
                    result: Err("Serial worker stopped unexpectedly.".into()),
                })
            }
        }
    }
}

impl Drop for SerialWorker {
    fn drop(&mut self) {
        // Never join a thread waiting on serial I/O from the UI thread.
        self.stopping.store(true, Ordering::Relaxed);
    }
}

fn run_worker(
    incoming: Receiver<SerialCommand>,
    outgoing: Sender<SerialReply>,
    ctx: egui::Context,
    stopping: Arc<AtomicBool>,
) {
    let mut connection: Option<Connection> = None;
    while let Ok(command) = incoming.recv() {
        if stopping.load(Ordering::Relaxed) {
            break;
        }
        let operation = command.label();
        let result = match command {
            SerialCommand::ListPorts => Ok(SerialOutcome::Ports(SerialManager::enumerate_and_filter_ports())),
            SerialCommand::Connect(port) => {
                connection = None;
                Connection::connect(port).map(|(new_connection, context, update)| {
                    connection = Some(new_connection);
                    SerialOutcome::Connected(context, update)
                })
            }
            SerialCommand::Disconnect => {
                connection = None;
                Ok(SerialOutcome::Disconnected)
            }
            command => match connection.as_mut() {
                Some(connection) => connection.execute(command).map(SerialOutcome::Update),
                None => Err(anyhow!("No CPU is connected.")),
            },
        }
        .map_err(|error| format!("{error:#}"));
        if outgoing.send(SerialReply { operation, result }).is_err() {
            break;
        }
        ctx.request_repaint();
    }
}

struct Connection {
    client: CpuClient,
    cpu_type: ServerCpuType,
    unsupported_pins: Vec<CpuPin>,
    loaded_regs: RemoteCpuRegisters,
    readout_state: Option<ProgramState>,
    trap_readout: Option<(u32, u32, u32, TrapStatus)>,
}

impl Connection {
    fn connect(port: String) -> Result<(Self, ClientContext, SerialUpdate)> {
        let mut client = CpuClient::init(Some(port.clone()), None)?;
        let (cpu_type, queue_status) = client.cpu_type()?;
        let state = client.get_program_state()?;
        let flags = client.get_flags()?;
        let mut context = ClientContext::new(port, cpu_type, queue_status, state, flags);
        context.baud_rate = client.baud_rate().ok();
        let mut connection = Self {
            client,
            cpu_type,
            unsupported_pins: Vec::new(),
            loaded_regs: context.initial_state().regs.clone(),
            readout_state: None,
            trap_readout: None,
        };
        let update = SerialUpdate {
            pins: connection.read_pins(),
            ..Default::default()
        };
        Ok((connection, context, update))
    }

    fn read_pins(&mut self) -> Vec<PinReadout> {
        let pins = if matches!(self.cpu_type, ServerCpuType::Intel80386) {
            [CpuPin::READY, CpuPin::HOLD]
        }
        else {
            [CpuPin::READY, CpuPin::TEST]
        };
        let mut result = Vec::new();
        for pin in pins {
            if self.unsupported_pins.contains(&pin) {
                continue;
            }
            let value = self.client.read_pin(pin);
            if matches!(value, Err(CpuClientError::CommandFailed(_))) {
                self.unsupported_pins.push(pin);
            }
            result.push(PinReadout { pin, value });
        }
        result
    }

    fn set_flag(&mut self, flag: u32, enabled: bool) -> Result<u32> {
        if flag == ServerFlags::ENABLE_TRAP_EXECUTION && enabled {
            self.client.enable_trap_execution()?;
            return Ok(self.client.get_flags()?);
        }
        let flags = self.client.get_flags()?;
        let flags = if enabled { flags | flag } else { flags & !flag };
        self.client.set_flags(flags)?;
        Ok(flags)
    }

    fn execute(&mut self, command: SerialCommand) -> Result<SerialUpdate> {
        self.execute_inner(command).map_err(|error| {
            // Only query after a complete rejection reply. A transport failure
            // may have left the stream mid-command, so don't send another command.
            if matches!(error.downcast_ref::<CpuClientError>(), Some(CpuClientError::CommandFailed(_))) {
                match self.client.get_last_error() {
                    Ok(reason) if !reason.is_empty() => return error.context(format!("Server: {reason}")),
                    Err(reason_error) => log::warn!("Could not read server rejection reason: {reason_error}"),
                    _ => {}
                }
            }
            error
        })
    }

    fn read_trap_registers(&mut self, state: &InstructionState, update: &mut SerialUpdate) -> Result<()> {
        let available = matches!(state.status, TrapStatus::Paused | TrapStatus::Captured);
        // Retargeting a paused boundary can finalize the same image without
        // clocking. Publish that transition so the GUI changes comparison mode.
        let key = (state.run_id, state.sequence, state.capture_cycle, state.status);
        if !available || self.trap_readout == Some(key) {
            return Ok(());
        }
        let capture = self.client.trap_capture()?;
        update.registers = Some((self.loaded_regs.clone(), RemoteCpuRegisters::V2(capture.handler_registers)));
        self.trap_readout = Some(key);
        Ok(())
    }

    fn execute_inner(&mut self, command: SerialCommand) -> Result<SerialUpdate> {
        let mut update = SerialUpdate::default();
        match command {
            SerialCommand::Poll => {
                let status = self.client.server_status()?;
                let flags = self.client.get_flags()?;
                let trap = flags & ServerFlags::ENABLE_TRAP_EXECUTION != 0;
                if trap {
                    update.instruction = Some(self.client.instruction_state()?);
                }
                if matches!(status.state, ProgramState::StoreDone | ProgramState::StoreDoneSmm) {
                    if self.readout_state != Some(status.state) {
                        update.cycles = Some(self.client.get_cycle_states().context("Reading cycle log")?);
                        if !trap {
                            update.registers = Some((self.loaded_regs.clone(), self.client.store_registers().context("Reading final registers")?));
                        }
                    }
                }
                update.status = Some(status);
                update.flags = Some(flags);
                update.pins = self.read_pins();
                if let Some(state) = update.instruction.clone() {
                    self.read_trap_registers(&state, &mut update)?;
                }
                // Commit this only after the complete update can be delivered.
                self.readout_state = if matches!(status.state, ProgramState::StoreDone | ProgramState::StoreDoneSmm) {
                    Some(status.state)
                }
                else {
                    None
                };
            }
            SerialCommand::Load { mut regs, run, uploads } => {
                // Keep the entire upload/run transaction together; polling and
                // pin writes cannot consume any of its intermediate replies.
                self.upload_memory(uploads, &regs, &mut update)?;
                let state = self.client.get_program_state()?;
                update.flags = Some(self.set_flag(ServerFlags::EXECUTE_AUTOMATIC, run)?);
                ExecutionMode::from_flags(update.flags.unwrap()).apply_to_registers(&mut regs);
                let initial_regs = regs.clone();
                if !run && state == ProgramState::StoreDoneSmm {
                    regs = regs
                        .to_b()
                        .ok_or_else(|| anyhow!("Cannot convert registers for SMM loading."))?;
                }
                let mut buffer = Cursor::new(Vec::new());
                regs.write(&mut buffer).context("Serializing registers")?;
                self.client
                    .load_registers_from_buf(RegisterSetType::from(&regs), buffer.get_ref())
                    .context("Loading registers")?;
                self.loaded_regs = initial_regs;
                update.loaded_regs = Some(self.loaded_regs.clone());
                if update.flags.unwrap_or(0) & ServerFlags::ENABLE_TRAP_EXECUTION != 0 {
                    update.instruction = Some(self.client.instruction_state()?);
                }
                self.readout_state = None;
                self.trap_readout = None;
                update.status = Some(self.client.server_status()?);
                if !run {
                    update.cycle = Some(self.client.get_cycle_state(false)?);
                }
                update.pins = self.read_pins();
                update.notice = Some(
                    if run {
                        "Program started."
                    }
                    else {
                        "Registers loaded successfully!"
                    }
                    .into(),
                );
            }
            SerialCommand::Step => {
                update.cycle = Some(self.client.get_cycle_state(true)?);
                let flags = self.client.get_flags()?;
                update.flags = Some(flags);
                if flags & ServerFlags::ENABLE_TRAP_EXECUTION != 0 {
                    update.instruction = Some(self.client.instruction_state()?);
                }
                update.pins = self.read_pins();
            }
            SerialCommand::StepInstruction | SerialCommand::ContinueTrap => {
                let state = if matches!(command, SerialCommand::StepInstruction) {
                    self.client.step_instruction(100_000)?
                } else {
                    self.client.continue_trap(100_000)?
                };
                if matches!(command, SerialCommand::ContinueTrap) {
                    update.notice = Some(format!("Trap execution: {:?} ({:?})", state.status, state.reason));
                }
                update.status = Some(self.client.server_status()?);
                self.read_trap_registers(&state, &mut update)?;
                update.instruction = Some(state);
            }
            SerialCommand::ResetCpu => {
                self.client.reset_cpu()?;
                update.trap_endpoint = Some(None);
                self.readout_state = None;
                self.trap_readout = None;
                let flags = self.client.get_flags()?;
                update.flags = Some(flags);
                if flags & ServerFlags::ENABLE_TRAP_EXECUTION != 0 {
                    update.instruction = Some(self.client.instruction_state()?);
                }
                update.status = Some(self.client.server_status()?);
            }
            SerialCommand::SetTrapEndpoint { cs, ip, enabled } => {
                let endpoint = enabled.then_some(InstructionAddress { cs, ip });
                self.client.set_trap_endpoint(endpoint)?;
                update.trap_endpoint = Some(endpoint);
            }
            SerialCommand::SetFlag { flag, enabled } => {
                update.flags = Some(self.set_flag(flag, enabled)?);
            }
            SerialCommand::SetExecutionMode(mode) => {
                let flags = self.client.get_flags()?;
                if let Some(reason) = mode.unavailable_reason(self.cpu_type, flags) {
                    return Err(anyhow!(reason));
                }
                update.flags = Some(self.set_flag(ServerFlags::ENABLE_TRAP_EXECUTION, mode == ExecutionMode::Trap)?);
                if mode == ExecutionMode::Trap {
                    update.instruction = Some(self.client.instruction_state()?);
                    update.status = Some(self.client.server_status()?);
                }
            }
            SerialCommand::WritePin { pin, value } => {
                self.client.write_pin(pin, value)?;
                update.pins.push(PinReadout {
                    pin,
                    value: self.client.read_pin(pin),
                });
            }
            SerialCommand::WriteDataBus(value) => {
                self.client.write_data_bus(value)?;
            }
            SerialCommand::ReadMemory { address, size } => {
                let mut data = Vec::new();
                self.client.read_memory(address, size, &mut data)?;
                update.memory = Some((address, data));
            }
            SerialCommand::ReadDisassembly(request) => {
                update.disassembly = Some(read_disassembly(&mut self.client, self.cpu_type, request)?);
            }
            SerialCommand::Upload { upload, regs } => {
                update.notice = Some(format!("Binary {} uploaded.", upload.name));
                self.upload_memory(vec![upload], &regs, &mut update)?;
            }
            SerialCommand::EraseMemory => {
                self.client.erase_memory()?;
                update.memory_changed = true;
                update.notice = Some("Memory erased successfully!".into());
            }
            SerialCommand::ClearCycleLog => {
                self.client.clear_cycle_log()?;
                update.cycles = Some(Vec::new());
            }
            SerialCommand::ListPorts | SerialCommand::Connect(_) | SerialCommand::Disconnect => unreachable!(),
        }
        Ok(update)
    }

    fn upload_memory(
        &mut self,
        uploads: Vec<MemoryUpload>,
        regs: &RemoteCpuRegisters,
        update: &mut SerialUpdate,
    ) -> Result<()> {
        if uploads.is_empty() {
            return Ok(());
        }
        let flags = self.client.get_flags()?;
        let mut endpoint = None;
        // Every upload action uses this preparation and transfer sequence.
        // Validate the complete batch before modifying server memory.
        for upload in &uploads {
            if let Some(address) = upload.trap_endpoint(self.cpu_type, regs, flags)? {
                endpoint = Some(address);
            }
        }
        for upload in uploads {
            self.client
                .set_memory(upload.address, &upload.data)
                .with_context(|| format!("Uploading {} at {:06X}", upload.name, upload.address))?;
        }
        if let Some(endpoint) = endpoint {
            self.client.set_trap_endpoint(Some(endpoint))?;
            update.trap_endpoint = Some(Some(endpoint));
        }
        update.memory_changed = true;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::{events::GuiEventQueue, windows::ClientWindow};
    use std::time::{Duration, Instant};

    fn receive(worker: &mut SerialWorker) -> SerialReply {
        let deadline = Instant::now() + Duration::from_secs(3);
        loop {
            if let Some(reply) = worker.try_recv() {
                return reply;
            }
            assert!(Instant::now() < deadline, "Worker did not complete");
            thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn polling_preserves_ui_and_defers_work() {
        let ctx = egui::Context::default();
        ctx.style_mut(|style| style.animation_time = 0.0);
        let (started_tx, started_rx) = crossbeam_channel::bounded(1);
        let (release_tx, release_rx) = crossbeam_channel::bounded(1);
        let mut worker = SerialWorker::spawn(ctx.clone(), move |requests, replies, _, _| {
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Poll));
            started_tx.send(()).unwrap();
            release_rx.recv_timeout(Duration::from_secs(3)).unwrap();
            replies
                .send(SerialReply {
                    operation: "Reading CPU status",
                    result:    Ok(SerialOutcome::Update(Default::default())),
                })
                .unwrap();
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Step));
            replies
                .send(SerialReply {
                    operation: "Stepping CPU",
                    result:    Ok(SerialOutcome::Update(Default::default())),
                })
                .unwrap();
        });
        let context = ClientContext::new(
            "test".into(),
            ServerCpuType::Intel80286,
            false,
            ProgramState::Execute,
            0,
        );
        let mut window = ClientWindow::default();
        window.init(&context);
        let mut events = GuiEventQueue::default();
        let mut render = |worker: &SerialWorker| {
            ctx.run(
                egui::RawInput {
                    time: Some(1.0),
                    ..Default::default()
                },
                |ctx| {
                    window.show(ctx, &context, &mut events, worker.controls_busy());
                },
            )
            .shapes
        };
        for _ in 0..3 {
            render(&worker);
        }
        let idle = render(&worker);
        worker.submit(SerialCommand::Poll).unwrap();
        started_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        worker.pending_since = Instant::now() - Duration::from_secs(1);
        assert!(worker.busy());
        assert!(!worker.controls_busy());
        assert!(worker.activity().is_none());
        assert_eq!(
            idle,
            render(&worker),
            "Polling changed the window's rendered appearance"
        );

        worker.submit(SerialCommand::Step).unwrap();
        assert!(worker.controls_busy());
        assert!(worker.submit(SerialCommand::EraseMemory).is_err());
        assert!(worker.submit(SerialCommand::Poll).is_err());
        release_tx.send(()).unwrap();
        let reply = receive(&mut worker);
        assert_eq!(reply.operation, "Stepping CPU");
        assert!(reply.result.is_ok());
        assert!(!worker.controls_busy());
    }

    #[test]
    fn failed_read_cancels_queued_action() {
        let (observed_tx, observed_rx) = crossbeam_channel::unbounded();
        let mut worker = SerialWorker::spawn(egui::Context::default(), move |requests, replies, _, _| {
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Poll));
            replies
                .send(SerialReply {
                    operation: "Reading CPU status",
                    result:    Err("Read timeout".into()),
                })
                .unwrap();
            while let Ok(command) = requests.recv() {
                observed_tx.send(command.label()).unwrap();
            }
        });
        worker.submit(SerialCommand::Poll).unwrap();
        worker.submit(SerialCommand::Step).unwrap();
        let reply = receive(&mut worker);
        assert_eq!(reply.operation, "Stepping CPU");
        assert!(reply.result.is_err());
        assert!(!worker.busy());
        assert!(observed_rx.try_recv().is_err());
    }

    #[test]
    fn disconnect_cancels_queued_actions() {
        let mut worker = SerialWorker::spawn(egui::Context::default(), move |requests, replies, _, _| {
            assert!(matches!(requests.recv().unwrap(), SerialCommand::ReadMemory { .. }));
            replies
                .send(SerialReply {
                    operation: "Reading memory",
                    result:    Ok(SerialOutcome::Update(Default::default())),
                })
                .unwrap();
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Disconnect));
            replies
                .send(SerialReply {
                    operation: "Disconnecting",
                    result:    Ok(SerialOutcome::Disconnected),
                })
                .unwrap();
        });
        worker
            .submit_background(SerialCommand::ReadMemory { address: 0, size: 4 })
            .unwrap();
        assert!(!worker.controls_busy());
        worker.submit(SerialCommand::Step).unwrap();
        worker.disconnect().unwrap();
        assert!(matches!(receive(&mut worker).result, Ok(SerialOutcome::Disconnected)));
        assert!(!worker.busy());
    }

    #[test]
    fn pending_io_keeps_frames_responsive() {
        let ctx = egui::Context::default();
        let (started_tx, started_rx) = crossbeam_channel::bounded(1);
        let (release_tx, release_rx) = crossbeam_channel::bounded(1);
        let mut worker = SerialWorker::spawn(ctx.clone(), move |requests, replies, _, _| {
            let command = requests.recv().unwrap();
            started_tx.send(()).unwrap();
            release_rx.recv_timeout(Duration::from_secs(3)).unwrap();
            replies
                .send(SerialReply {
                    operation: command.label(),
                    result:    Ok(SerialOutcome::Update(SerialUpdate {
                        notice: Some("Stale result".into()),
                        ..Default::default()
                    })),
                })
                .unwrap();
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Disconnect));
            replies
                .send(SerialReply {
                    operation: "Disconnecting",
                    result:    Ok(SerialOutcome::Disconnected),
                })
                .unwrap();
        });
        worker.submit(SerialCommand::Step).unwrap();
        started_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        let context = ClientContext::new(
            "test".into(),
            ServerCpuType::Intel80286,
            false,
            ProgramState::Execute,
            0,
        );
        let mut window = ClientWindow::default();
        window.init(&context);
        let mut events = GuiEventQueue::default();
        let start = Instant::now();
        for _ in 0..10 {
            assert!(worker.try_recv().is_none());
            assert!(worker.submit(SerialCommand::Poll).is_err());
            let output = ctx.run(egui::RawInput::default(), |ctx| {
                window.show(ctx, &context, &mut events, worker.busy());
            });
            assert!(!output.shapes.is_empty());
        }
        assert!(start.elapsed() < Duration::from_secs(1), "Rendering waited for I/O");
        worker.disconnect().unwrap();
        assert!(worker.submit(SerialCommand::Connect("new port".into())).is_err());
        release_tx.send(()).unwrap();
        assert!(matches!(receive(&mut worker).result, Ok(SerialOutcome::Disconnected)));
        assert!(!worker.busy());
    }

    #[test]
    fn errors_clear_busy_and_preserve_order() {
        let mut worker = SerialWorker::spawn(egui::Context::default(), |requests, replies, _, _| {
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Step));
            replies
                .send(SerialReply {
                    operation: "Stepping CPU",
                    result:    Err("Read timeout".into()),
                })
                .unwrap();
            assert!(matches!(requests.recv().unwrap(), SerialCommand::Poll));
            replies
                .send(SerialReply {
                    operation: "Reading CPU status",
                    result:    Ok(SerialOutcome::Update(Default::default())),
                })
                .unwrap();
        });
        worker.submit(SerialCommand::Step).unwrap();
        assert!(receive(&mut worker).result.is_err());
        assert!(!worker.busy());
        worker.submit(SerialCommand::Poll).unwrap();
        assert!(receive(&mut worker).result.is_ok());
    }

    #[test]
    fn closing_does_not_wait_for_io() {
        let (started_tx, started_rx) = crossbeam_channel::bounded(1);
        let (release_tx, release_rx) = crossbeam_channel::bounded(1);
        let (finished_tx, finished_rx) = crossbeam_channel::bounded(1);
        let mut worker = SerialWorker::spawn(egui::Context::default(), move |requests, _, _, stopping| {
            requests.recv().unwrap();
            started_tx.send(()).unwrap();
            release_rx.recv_timeout(Duration::from_secs(2)).unwrap();
            assert!(stopping.load(Ordering::Relaxed));
            finished_tx.send(()).unwrap();
        });
        worker.submit(SerialCommand::Poll).unwrap();
        started_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        let start = Instant::now();
        drop(worker);
        assert!(start.elapsed() < Duration::from_millis(100));
        release_tx.send(()).unwrap();
        finished_rx.recv_timeout(Duration::from_secs(1)).unwrap();
    }

    #[test]
    #[ignore = "requires ARDUINOX86_PORT and exclusive access to the physical 286; replaces emulated memory"]
    fn hardware_installed_trap_handler() -> Result<()> {
        use anyhow::ensure;
        use arduinox86_client::{trap::*, MemoryStrategy, RemoteCpuRegistersV2, ServerCapabilities};
        let (mut connection, _, _) = Connection::connect(std::env::var("ARDUINOX86_PORT")?)?;
        let result = (|| -> Result<()> {
            connection.client.reset_cpu()?;
            ensure!(connection.client.capabilities() & ServerCapabilities::SERVER_TRAP_HANDLER_286 != 0);
            ensure!(connection.client.capabilities() & ServerCapabilities::TRAP_EXECUTION_286 != 0);
            ensure!(connection.client.set_real_mode_trap_handler(0x800).is_err());
            ensure!(connection.client.get_last_error()?.contains("STOREALL"));
            for backend in [0, ServerFlags::USE_SDRAM_BACKEND] {
                connection.client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING | backend)?;
                connection.client.erase_memory()?;
                connection.client.set_memory_strategy(MemoryStrategy::Zero, 0, 0x1000000)?;
                connection.client.enable_trap_execution()?;
                for custom in [false, true] {
                    connection.client.reset_cpu()?;
                    let handler = if custom { 0x005111 } else { DEFAULT_REAL_MODE_TRAP_HANDLER };
                    let idt = if custom { 0x003A01 } else { MANAGED_IDT_BASE };
                    connection.client.set_real_mode_trap_handler(handler)?;
                    let poison = [0xF4; 16];
                    connection.client.set_memory(handler, &poison)?;
                    connection.client.set_memory(idt, &[0xA5; 12])?;
                    connection.client.set_memory(0x010100, &[0xB8, 0x34, 0x12, 0x40, 0x43, 0x90])?;
                    connection.client.set_trap_endpoint(Some(InstructionAddress { cs: 0x1000, ip: 0x105 }))?;
                    let mut regs = RemoteCpuRegistersV2 {
                        cs: 0x1000, ip: 0x100, ss: 0x0800, sp: 0x1000,
                        ..Default::default()
                    };
                    regs.normalize_descriptors();
                    regs.idt_desc.set_base_address(idt);
                    regs.idt_desc.set_limit(MANAGED_IDT_LIMIT_REAL);
                    if custom {
                        let mut bytes = Vec::new();
                        regs.to_buffer(&mut bytes);
                        connection.client.load_registers_from_buf(RegisterSetType::Intel286, &bytes)?;
                    } else {
                        let loaded = connection.execute(SerialCommand::Load {
                            regs: RemoteCpuRegisters::V2(regs), run: false, uploads: Vec::new(),
                        })?;
                        let RemoteCpuRegisters::V2(regs) = loaded.loaded_regs.context("loaded registers")? else {
                            anyhow::bail!("expected 286 registers");
                        };
                        ensure!(regs.idt_desc.base_address() == MANAGED_IDT_BASE);
                        ensure!(regs.idt_desc.limit() == MANAGED_IDT_LIMIT_REAL);
                    }
                    let ready = connection.client.instruction_state()?;
                    ensure!(ready.status == TrapStatus::Ready && ready.handler_address == handler);
                    let mut vector = Vec::new();
                    connection.client.read_memory(idt, 12, &mut vector)?;
                    ensure!(vector[..4] == [0xA5; 4] && vector[8..] == [0xA5; 4]);
                    let offset = u16::from_le_bytes([vector[4], vector[5]]);
                    let segment = u16::from_le_bytes([vector[6], vector[7]]);
                    ensure!(u32::from(segment) * 16 + u32::from(offset) == handler);
                    let update = connection.execute(SerialCommand::StepInstruction)?;
                    let (_, RemoteCpuRegisters::V2(raw)) = update.registers.context("step register readout")? else {
                        anyhow::bail!("expected 286 step registers");
                    };
                    ensure!(raw.ax == 0x1234);
                    ensure!(update.notice.is_none(), "step emitted a toast");
                    let first = update.instruction.context("first step")?;
                    ensure!(first.status == TrapStatus::Paused && first.resume_ip == 0x103, "first step: {first:?}");
                    ensure!(first.capture_cycle > first.trap_cycle);
                    let cycle = connection.client.server_status()?.cycle_ct;
                    let mut snapshot = Vec::new();
                    raw.to_buffer(&mut snapshot);
                    // Recovery and readback must use the captured image, even if
                    // a memory editor overwrites the fixed LOADALL workspace.
                    connection.client.set_memory(0x800, &[0xA5; 102])?;
                    let capture = connection.client.trap_capture()?;
                    let mut again = Vec::new();
                    capture.handler_registers.to_buffer(&mut again);
                    ensure!(snapshot == again && capture.instruction == first);
                    ensure!(connection.execute(SerialCommand::Poll)?.registers.is_none(), "repeated poll reread registers");
                    ensure!(connection.client.server_status()?.cycle_ct == cycle, "paused queries clocked CPU");
                    ensure!(connection.client.set_real_mode_trap_handler(0x5200).is_err());
                    ensure!(connection.client.get_last_error()?.contains("Reset or finish"));
                    let update = connection.execute(SerialCommand::StepInstruction)?;
                    let second = update.instruction.context("second step")?;
                    if second.status != TrapStatus::Paused {
                        std::fs::write(std::env::temp_dir().join("arduinox86-ice-failure.txt"),
                            format!("{second:?}\n{}\n{:#?}", connection.client.get_last_error()?, connection.client.get_cycle_states()?))?;
                    }
                    ensure!(second.status == TrapStatus::Paused && second.resume_ip == 0x104, "second step: {second:?}");
                    let (_, RemoteCpuRegisters::V2(raw)) = update.registers.context("second step register readout")? else {
                        anyhow::bail!("expected 286 step registers");
                    };
                    ensure!(raw.ax == 0x1235 && raw.bx == 0);
                    let last = connection.execute(SerialCommand::ContinueTrap)?.instruction.context("endpoint")?;
                    ensure!(last.status == TrapStatus::Captured && last.resume_ip == 0x105, "endpoint: {last:?}");
                    let raw = connection.client.trap_capture()?.handler_registers;
                    ensure!(raw.ax == 0x1235 && raw.bx == 1);
                    let mut memory = Vec::new();
                    connection.client.read_memory(handler, 16, &mut memory)?;
                    ensure!(memory == poison, "handler backing memory changed");
                    println!("PASS backend={backend:#X} IDTR={idt:06X} handler={handler:06X}: capture/restore/step + endpoint");
                }
                // Legacy loads must not install or alter the trap vector.
                connection.client.reset_cpu()?;
                connection.client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING | backend)?;
                connection.client.set_memory(MANAGED_IDT_BASE + 4, &[0xA5; 4])?;
                let mut regs = RemoteCpuRegistersV2 { cs: 0x1000, ip: 0x100, ..Default::default() };
                regs.normalize_descriptors();
                connection.execute(SerialCommand::Load { regs: RemoteCpuRegisters::V2(regs), run: false, uploads: Vec::new() })?;
                let mut vector = Vec::new();
                connection.client.read_memory(MANAGED_IDT_BASE + 4, 4, &mut vector)?;
                ensure!(vector == [0xA5; 4]);
                connection.client.reset_cpu()?;
            }
            Ok(())
        })();
        let reset = connection.client.reset_cpu();
        let address = connection.client.set_real_mode_trap_handler(DEFAULT_REAL_MODE_TRAP_HANDLER);
        let flags = connection.client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING);
        result?;
        reset?;
        address?;
        flags?;
        Ok(())
    }

    /// Use a fixture emitted by test_generator's protected_mode example.
    /// This replaces emulated memory and resets the physical CPU on completion.
    #[test]
    #[ignore = "requires ARDUINOX86_PORT and ARDUINOX86_PM_FIXTURE, with exclusive access to the physical 286"]
    fn hardware_disassembly_follows_step() -> Result<()> {
        use crate::windows::DisassemblyWindow;
        use arduinox86_client::{trap::{InstructionAddress, TrapStatus}, BinWrite, MemoryStrategy, RemoteCpuRegistersV2};
        use anyhow::ensure;
        let port = std::env::var("ARDUINOX86_PORT")?;
        let fixture: serde_json::Value = serde_json::from_slice(&std::fs::read(std::env::var("ARDUINOX86_PM_FIXTURE")?)?)?;
        let (mut connection, _, _) = Connection::connect(port)?;
        let result = (|| -> Result<()> {
            connection.client.reset_cpu()?;
            let backend = if std::env::var("ARDUINOX86_USE_SDRAM").as_deref() == Ok("1") {
                ServerFlags::USE_SDRAM_BACKEND
            } else {
                0
            };
            connection.client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING | backend)?;
            connection.client.enable_trap_execution()?;
            let before = connection.client.server_status()?.cycle_ct;
            let rejected = connection.execute(SerialCommand::StepInstruction).err().context("unloaded step must fail")?;
            ensure!(format!("{rejected:#}").contains("and an endpoint for continue"), "Full server rejection was not reported: {rejected:#}");
            ensure!(connection.client.server_status()?.cycle_ct == before, "rejected step clocked CPU");
            connection.client.erase_memory()?;
            connection.client.set_memory_strategy(MemoryStrategy::Zero, 0, 0x1000000)?;
            for region in fixture["initial_memory"].as_array().context("fixture memory")? {
                let data: Vec<u8> = serde_json::from_value(region["bytes"].clone())?;
                connection.client.set_memory(region["address"].as_u64().context("memory address")? as u32, &data)?;
            }
            connection.client.set_trap_endpoint(Some(InstructionAddress {
                cs: fixture["endpoint"][0].as_u64().context("endpoint CS")? as u16,
                ip: fixture["endpoint"][1].as_u64().context("endpoint IP")? as u16,
            }))?;
            let bytes: Vec<u8> = serde_json::from_value(fixture["initial_loadall"].clone())?;
            let mut regs = RemoteCpuRegistersV2::try_from(bytes.as_slice()).map_err(anyhow::Error::msg)?;
            regs.flags &= !arduinox86_client::trap::TRAP_FLAG;
            // Use the managed handler slot while retaining this fixture's ring-3
            // stacks and deliberately non-paragraph-aligned guest descriptors.
            use arduinox86_client::trap::{MANAGED_HANDLER_BASE, DEFAULT_REAL_MODE_TRAP_HANDLER};
            let handler_desc = [0xFF, 0x0F, 0x00, 0x50, 0x00, 0x9B, 0x00, 0x00];
            connection.client.set_memory(regs.gdt_desc.base_address() + 8, &handler_desc)?;
            let gate = [0x10, 0x00, 0x08, 0x00, 0x00, 0x86, 0x00, 0x00];
            connection.client.set_memory(regs.idt_desc.base_address() + 8, &gate)?;
            let poison = [0xF4; 16];
            connection.client.set_memory(DEFAULT_REAL_MODE_TRAP_HANDLER, &poison)?;
            ensure!(DEFAULT_REAL_MODE_TRAP_HANDLER == MANAGED_HANDLER_BASE + 0x10);
            // Bypass GUI normalization to verify that firmware owns TF as well.
            let mut raw = Cursor::new(Vec::new());
            regs.write(&mut raw)?;
            connection.client.load_registers_from_buf(RegisterSetType::Intel286, raw.get_ref())?;
            ensure!(connection.client.instruction_state()?.flags & arduinox86_client::trap::TRAP_FLAG != 0);
            let base = regs.cs_desc.base_address();
            let loaded = connection.execute(SerialCommand::Load { regs: RemoteCpuRegisters::V2(regs), run: false, uploads: Vec::new() })?;
            ensure!(loaded.loaded_regs.as_ref().context("load acknowledgement")?.flags() & arduinox86_client::trap::TRAP_FLAG != 0);
            ensure!(loaded.status.context("load status")?.state == ProgramState::Execute);
            ensure!(loaded.instruction.as_ref().context("load instruction")?.handler_address == DEFAULT_REAL_MODE_TRAP_HANDLER);
            let mut view = DisassemblyWindow::default();
            view.connect(connection.cpu_type);
            view.loaded(loaded.loaded_regs.as_ref().context("load acknowledgement")?);
            view.observe(loaded.instruction.as_ref().context("initial instruction")?);
            for _ in 0..2 {
                let request = view.pending_request().context("disassembly request")?;
                view.submitted(request);
                let before = connection.client.instruction_state()?;
                let cycle = connection.client.server_status()?.cycle_ct;
                let mut read = connection.execute(SerialCommand::ReadDisassembly(request))?;
                let readout = read.disassembly.take().context("disassembly reply")?;
                let rows = readout.result.as_ref().map_err(|e| anyhow!("{e}"))?;
                ensure!(!rows.is_empty());
                let current = rows.iter().find(|row| row.address == base + u32::from(before.resume_ip))
                    .context("current instruction in retained listing")?;
                ensure!(current.ip == u32::from(before.resume_ip));
                ensure!(!current.instruction.starts_with("db "));
                ensure!(before == connection.client.instruction_state()?, "read changed trap state");
                ensure!(cycle == connection.client.server_status()?.cycle_ct, "read clocked CPU");
                println!("{:04X}:{:04X} @ {:06X}: {}", before.resume_cs, before.resume_ip, current.address, current.instruction);
                view.apply(readout);
                view.execution_started();
                let stepped = connection.execute(SerialCommand::StepInstruction)?;
                let next = stepped.instruction.as_ref().context("step boundary")?;
                ensure!(matches!(next.status, TrapStatus::Paused | TrapStatus::Captured));
                ensure!(next.sequence > before.sequence);
                view.observe(next);
            }
            let mut memory = Vec::new();
            connection.client.read_memory(DEFAULT_REAL_MODE_TRAP_HANDLER, 16, &mut memory)?;
            ensure!(memory == poison, "handler instructions overwrote backing memory");
            Ok(())
        })();
        let reset = connection.client.reset_cpu();
        let flags = connection.client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING);
        result?;
        reset?;
        flags?;
        Ok(())
    }
}
