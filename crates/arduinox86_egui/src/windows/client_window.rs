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
    events::{GuiEvent, GuiEventQueue},
    serial_worker::SerialUpdate,
};
use arduinox86_client::{ServerFlags, ServerStatus};
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
}

impl ClientWindow {
    pub fn init(&mut self, c_ctx: &ClientContext) {
        self.reset_state();
        self.cycle_table.set_arch(c_ctx.cpu_type);
        self.cpu_inputs = CpuInputControls::new(c_ctx.cpu_type);
    }

    pub fn reset_state(&mut self) {
        self.server_status = None;
        self.last_status_time = None;
        self.last_cycle_ct = 0;
        self.effective_mhz = 0.0;
        self.cycle_table.clear();
    }

    pub fn apply(&mut self, update: &mut SerialUpdate) {
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

    pub fn show(&mut self, e_ctx: &egui::Context, c_ctx: &ClientContext, events: &mut GuiEventQueue, busy: bool) {
        egui::Window::new("Client Connection")
            .default_width(800.0)
            .default_height(600.0)
            .show(e_ctx, |ui| {
                egui::MenuBar::new().ui(ui, |ui| {
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
                        (
                            egui_phosphor::regular::BOX_ARROW_UP,
                            "Load registers",
                            GuiEvent::LoadRegisters,
                        ),
                        (egui_phosphor::fill::ERASER, "Erase Memory", GuiEvent::EraseMemory),
                        (egui_phosphor::fill::PLAY_PAUSE, "Step", GuiEvent::Step),
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
                        .button(egui::RichText::new(egui_phosphor::regular::PLUGS).size(24.0))
                        .on_hover_text("Disconnect")
                        .clicked()
                    {
                        events.push(GuiEvent::Disconnect);
                    }
                });
                ui.separator();
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
