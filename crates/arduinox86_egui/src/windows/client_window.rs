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
use crate::{
    client::ClientContext,
    controls::{cpu_inputs::CpuInputControls, cycle_table::CycleTable},
    enums::ExecutionMode,
    events::{GuiEvent, GuiEventQueue},
    serial_worker::SerialUpdate,
};
use arduinox86_client::{trap::TrapStatus, ProgramState, ServerFlags, ServerStatus};
use egui::{
    containers::menu::{MenuButton, MenuConfig},
    PopupCloseBehavior,
};
use std::time::Instant;

#[derive(Default)]
pub struct ClientWindow {
    last_status_time: Option<Instant>,
    last_cycle_ct: u64,
    server_status: Option<ServerStatus>,
    effective_mhz: f32,
    cycle_table: CycleTable,
    cpu_inputs: CpuInputControls,
    instruction: Option<arduinox86_client::trap::InstructionState>,
    endpoint_cs: u16,
    endpoint_ip: u16,
    endpoint_enabled: bool,
}

impl ClientWindow {
    pub fn init(&mut self, c_ctx: &ClientContext) {
        self.reset_state();
        self.cycle_table.set_arch(c_ctx.cpu_type);
        self.cpu_inputs = CpuInputControls::new(c_ctx.cpu_type);
    }

    pub fn reset_state(&mut self) {
        self.server_status = None;
        self.instruction = None;
        self.last_status_time = None;
        self.last_cycle_ct = 0;
        self.effective_mhz = 0.0;
        self.cycle_table.clear();
    }

    pub fn apply(&mut self, update: &mut SerialUpdate) {
        if let Some(endpoint) = update.trap_endpoint {
            self.endpoint_enabled = endpoint.is_some();
            if let Some(endpoint) = endpoint {
                self.endpoint_cs = endpoint.cs;
                self.endpoint_ip = endpoint.ip;
            }
        }
        if let Some(instruction) = update.instruction.take() { self.instruction = Some(instruction); }
        if let Some(status) = update.status {
            let now = Instant::now();
            self.effective_mhz = self
                .last_status_time
                .map(|last| {
                    let elapsed = now.duration_since(last).as_secs_f32();
                    if elapsed > 0.0 {
                        status.cycle_ct.saturating_sub(self.last_cycle_ct) as f32 / elapsed / 1_000_000.0
                    }
                    else {
                        0.0
                    }
                })
                .unwrap_or(0.0);
            self.last_cycle_ct = status.cycle_ct;
            self.last_status_time = Some(now);
            self.server_status = Some(status);
        }
        if let Some(cycles) = update.cycles.take() {
            self.cycle_table.set_cycles(cycles);
        }
        if let Some(cycle) = update.cycle.take() {
            self.cycle_table.push_cycle(cycle);
        }
        self.cpu_inputs.apply(std::mem::take(&mut update.pins));
    }

    fn trap_run_disabled_reason(&self, c_ctx: &ClientContext) -> Option<&'static str> {
        let Some(instruction) = &self.instruction else {
            return Some("Waiting for the server's trap state.");
        };
        match instruction.status {
            TrapStatus::Idle => return Some(
                "Trap execution is not loaded. Load registers to install vector 1 and set TF. Protected mode requires a valid gate, descriptors and stack.",
            ),
            TrapStatus::Captured => return Some("Endpoint reached. Load registers to start another trap run."),
            TrapStatus::Failed => return Some("Trap execution failed. Reset CPU and reload registers before stepping."),
            _ => {}
        }
        if c_ctx.cached_flags() & ServerFlags::EXECUTE_AUTOMATIC != 0 {
            return Some("Pause automatic execution before stepping or continuing to the endpoint.");
        }
        if !matches!(c_ctx.program_state, ProgramState::Execute | ProgramState::TrapStoreDone) {
            return Some("The CPU is not ready for trap execution. Load registers before stepping.");
        }
        None
    }

    pub fn show(&mut self, e_ctx: &egui::Context, c_ctx: &ClientContext, events: &mut GuiEventQueue, busy: bool) {
        egui::Window::new("Client Connection")
            .default_width(800.0)
            .default_height(600.0)
            .show(e_ctx, |ui| {
                egui::MenuBar::new().ui(ui, |ui| {
                    MenuButton::new("Execution mode")
                        .config(MenuConfig::default().close_behavior(PopupCloseBehavior::CloseOnClickOutside))
                        .ui(ui, |ui| {
                            let flags = c_ctx.cached_flags();
                            let selected = ExecutionMode::from_flags(flags);
                            for mode in [ExecutionMode::QueueTracked, ExecutionMode::Trap, ExecutionMode::Halt] {
                                let unavailable = mode.unavailable_reason(c_ctx.cpu_type, flags);
                                let response = ui.add_enabled(
                                    !busy && unavailable.is_none(),
                                    egui::RadioButton::new(selected == mode, mode.label()),
                                );
                                if response.clicked() && selected != mode {
                                    events.push(GuiEvent::SetExecutionMode(mode));
                                }
                                if let Some(reason) = unavailable {
                                    response.on_disabled_hover_text(reason);
                                }
                                else {
                                    response.on_hover_text(
                                        "Applies when registers are loaded. Reset CPU first if a trap run is in progress.",
                                    );
                                }
                            }
                        });
                    MenuButton::new("Options")
                        .config(MenuConfig::default().close_behavior(PopupCloseBehavior::CloseOnClickOutside))
                        .ui(ui, |ui| {
                            for (flag, label) in [
                                (ServerFlags::ENABLE_DEBUG, "Enable Serial Debug"),
                                (ServerFlags::ENABLE_CYCLE_LOGGING, "Enable Cycle Logging"),
                                (ServerFlags::USE_SDRAM_BACKEND, "Use SDRAM Backend"),
                                (ServerFlags::USE_SMM, "Use SMM"),
                                (ServerFlags::RESOLVE_BUS_STEP, "Resolve bus when stepping"),
                                (ServerFlags::HALT_AFTER_JUMP, "HALT after jump"),
                                (ServerFlags::ENABLE_ALE_INTERRUPT, "ALE interrupt resets READY"),
                            ] {
                                let mut enabled = c_ctx.cached_flags() & flag != 0;
                                if ui
                                    .add_enabled(!busy, egui::Checkbox::new(&mut enabled, label))
                                    .changed()
                                {
                                    events.push(GuiEvent::SetFlag { flag, enabled });
                                }
                            }
                        });
                });
                ui.horizontal(|ui| {
                    for (icon, tooltip, event) in [
                        (egui_phosphor::fill::PLAY_PAUSE, "Step one cycle", GuiEvent::Step),
                        (egui_phosphor::fill::PLAY, "Run Autonomously", GuiEvent::RunProgram),
                        (
                            egui_phosphor::fill::PAUSE,
                            "Pause",
                            GuiEvent::SetFlag {
                                flag:    ServerFlags::EXECUTE_AUTOMATIC,
                                enabled: false,
                            },
                        ),
                        (
                            egui_phosphor::fill::STOP,
                            "Stop",
                            GuiEvent::SetFlag {
                                flag:    ServerFlags::EXECUTE_AUTOMATIC,
                                enabled: false,
                            },
                        ),
                    ] {
                        if ui
                            .add_enabled(!busy, egui::Button::new(egui::RichText::new(icon).size(24.0)))
                            .on_hover_text(tooltip)
                            .clicked()
                        {
                            events.push(event);
                        }
                    }
                    ui.separator();
                    if ui
                        .add_enabled(
                            !busy,
                            egui::Button::new(
                                egui::RichText::new(egui_phosphor::regular::ARROW_COUNTER_CLOCKWISE).size(24.0),
                            ),
                        )
                        .on_hover_text("Reset CPU")
                        .clicked()
                    {
                        events.push(GuiEvent::ResetCpu);
                    }
                    if ui
                        .button(egui::RichText::new(egui_phosphor::regular::PLUGS).size(24.0))
                        .on_hover_text("Disconnect")
                        .clicked()
                    {
                        events.push(GuiEvent::Disconnect);
                    }
                });
                ui.separator();
                if c_ctx.cached_flags() & ServerFlags::ENABLE_TRAP_EXECUTION != 0 {
                    let disabled_reason = self.trap_run_disabled_reason(c_ctx);
                    ui.add_enabled_ui(!busy, |ui| {
                        ui.horizontal(|ui| {
                            ui.add_enabled_ui(disabled_reason.is_none(), |ui| {
                                for (icon, tooltip, event) in [
                                    (egui_phosphor::fill::PLAY_PAUSE, "Step instruction", GuiEvent::StepInstruction),
                                    (egui_phosphor::fill::PLAY, "Continue to endpoint", GuiEvent::ContinueTrap),
                                ] {
                                    if ui
                                        .button(egui::RichText::new(icon).size(24.0))
                                        .on_hover_text(tooltip)
                                        .clicked()
                                    {
                                        events.push(event);
                                    }
                                }
                            });
                        });
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut self.endpoint_enabled, "Instruction endpoint")
                                .on_hover_text("Stop before this instruction. Program uploads select the final decoded instruction.");
                            ui.add(egui::DragValue::new(&mut self.endpoint_cs).hexadecimal(4, false, true));
                            ui.label(":");
                            ui.add(egui::DragValue::new(&mut self.endpoint_ip).hexadecimal(4, false, true));
                            if ui.button("Apply endpoint").clicked() {
                                events.push(GuiEvent::SetTrapEndpoint { cs: self.endpoint_cs, ip: self.endpoint_ip, enabled: self.endpoint_enabled });
                            }
                        });
                    });
                    if let Some(reason) = disabled_reason {
                        ui.label(reason);
                    }
                    if let Some(s) = &self.instruction {
                        if s.status != TrapStatus::Idle {
                            ui.label(format!("Server trap handler: {:06X}", s.handler_address));
                        }
                    }
                    ui.separator();
                }
                ui.label(format!(
                    "Connected to {} CPU on port {}",
                    c_ctx.cpu_type, c_ctx.port_name
                ));
                ui.separator();
                ui.columns(2, |columns| {
                    columns[0].horizontal(|ui| {
                        ui.label("Server state:");
                        ui.label(
                            self.server_status
                                .map(|status| format!("{:?}", status.state))
                                .unwrap_or("Uninitialized".into()),
                        );
                    });
                    columns[0].horizontal(|ui| {
                        ui.label("Cycle count:");
                        ui.label(self.server_status.map_or(0, |status| status.cycle_ct).to_string());
                        ui.separator();
                        ui.label(format!("Effective MHz: {:.2}", self.effective_mhz));
                    });
                    columns[0].horizontal(|ui| {
                        ui.label("Address latch:");
                        let address_digits = c_ctx.cpu_type.bus_chr_width();
                        ui.label(format!(
                            "{:0address_digits$X}",
                            self.server_status.map_or(0, |status| status.address_latch)
                        ));
                    });
                    columns[1].horizontal(|ui| {
                        ui.separator();
                        ui.vertical(|ui| self.cpu_inputs.show(ui, busy, events));
                    });
                });
                ui.separator();
                ui.add_enabled_ui(!busy, |ui| {
                    if let Some(response) = self.cycle_table.show(ui, events) {
                        if response.changed() {
                            if let Ok(value) = u16::from_str_radix(self.cycle_table.data_bus_str(), 16) {
                                events.push(GuiEvent::WriteDataBus(value));
                            }
                        }
                    }
                });
            });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arduinox86_client::{trap::InstructionState, ServerCpuType};

    #[test]
    fn shows_and_resets_uploaded_endpoint() {
        let mut window = ClientWindow::default();
        window.apply(&mut SerialUpdate {
            trap_endpoint: Some(Some(arduinox86_client::trap::InstructionAddress { cs: 0x18, ip: 0x104 })),
            ..Default::default()
        });
        assert!(window.endpoint_enabled);
        assert_eq!((window.endpoint_cs, window.endpoint_ip), (0x18, 0x104));
        // Resetting display history for Load must retain the configured endpoint.
        window.reset_state();
        assert!(window.endpoint_enabled);
        window.apply(&mut SerialUpdate { trap_endpoint: Some(None), ..Default::default() });
        assert!(!window.endpoint_enabled);
    }

    #[test]
    fn trap_step_requires_loaded_run() {
        let mut context = ClientContext::new(
            "test".into(), ServerCpuType::Intel80286, false,
            ProgramState::Execute, ServerFlags::ENABLE_TRAP_EXECUTION,
        );
        let mut window = ClientWindow::default();
        let mut bytes = [0; InstructionState::WIRE_SIZE];
        bytes[0] = 1;
        assert!(window.trap_run_disabled_reason(&context).is_some());
        window.instruction = Some(InstructionState::from_bytes(&bytes).unwrap());
        assert!(window.trap_run_disabled_reason(&context).unwrap().contains("not loaded"));
        for status in [TrapStatus::Ready, TrapStatus::Running, TrapStatus::Paused] {
            window.instruction.as_mut().unwrap().status = status;
            assert!(window.trap_run_disabled_reason(&context).is_none());
        }
        context.server_flags |= ServerFlags::EXECUTE_AUTOMATIC;
        assert!(window.trap_run_disabled_reason(&context).is_some());
        context.server_flags &= !ServerFlags::EXECUTE_AUTOMATIC;
        for status in [TrapStatus::Captured, TrapStatus::Failed, TrapStatus::Idle] {
            window.instruction.as_mut().unwrap().status = status;
            assert!(window.trap_run_disabled_reason(&context).is_some());
        }
    }
}
