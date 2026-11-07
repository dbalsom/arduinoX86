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

use crate::{client::ClientContext, serial_manager::SerialManager};

pub struct MemoryUpload {
    pub name:    String,
    pub address: u32,
    pub data:    Vec<u8>,
}

pub enum SerialCommand {
    ListPorts,
    Connect(String),
    Disconnect,
    Poll,
    Load { regs: RemoteCpuRegisters, run: bool, uploads: Vec<MemoryUpload> },
    Step,
    SetFlag { flag: u32, enabled: bool },
    WritePin { pin: CpuPin, value: bool },
    WriteDataBus(u16),
    ReadMemory { address: u32, size: u32 },
    Upload(MemoryUpload),
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
            Self::SetFlag { .. } => "Updating server options",
            Self::WritePin { .. } => "Updating CPU input",
            Self::WriteDataBus(_) => "Writing data bus",
            Self::ReadMemory { .. } => "Reading memory",
            Self::Upload(_) => "Uploading binary",
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
        let flags = self.client.get_flags()?;
        let flags = if enabled { flags | flag } else { flags & !flag };
        self.client.set_flags(flags)?;
        Ok(flags)
    }

    fn execute(&mut self, command: SerialCommand) -> Result<SerialUpdate> {
        let mut update = SerialUpdate::default();
        match command {
            SerialCommand::Poll => {
                let status = self.client.server_status()?;
                if matches!(status.state, ProgramState::StoreDone | ProgramState::StoreDoneSmm) {
                    if self.readout_state != Some(status.state) {
                        update.cycles = Some(self.client.get_cycle_states().context("Reading cycle log")?);
                        update.registers = Some((
                            self.loaded_regs.clone(),
                            self.client.store_registers().context("Reading final registers")?,
                        ));
                    }
                }
                update.status = Some(status);
                update.flags = Some(self.client.get_flags()?);
                update.pins = self.read_pins();
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
                for upload in uploads {
                    self.upload(upload)?;
                }
                let state = self.client.get_program_state()?;
                update.flags = Some(self.set_flag(ServerFlags::EXECUTE_AUTOMATIC, run)?);
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
                self.readout_state = None;
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
                update.pins = self.read_pins();
            }
            SerialCommand::SetFlag { flag, enabled } => {
                update.flags = Some(self.set_flag(flag, enabled)?);
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
            SerialCommand::Upload(upload) => {
                update.notice = Some(format!("Binary {} uploaded.", upload.name));
                self.upload(upload)?;
            }
            SerialCommand::EraseMemory => {
                self.client.erase_memory()?;
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

    fn upload(&mut self, upload: MemoryUpload) -> Result<()> {
        self.client
            .set_memory(upload.address, &upload.data)
            .with_context(|| format!("Uploading {} at {:06X}", upload.name, upload.address))?;
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
    fn polling_preserves_window_appearance_and_defers_one_user_action() {
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
    fn failed_background_read_cancels_a_queued_action() {
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
    fn disconnect_cancels_actions_queued_behind_background_memory_reads() {
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
    fn frames_keep_rendering_and_disconnect_discards_a_pending_result() {
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
    fn errors_release_busy_state_and_later_commands_remain_ordered() {
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
    fn closing_the_gui_does_not_join_blocked_io() {
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
}
