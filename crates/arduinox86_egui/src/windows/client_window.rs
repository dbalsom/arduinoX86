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
use std::time::Instant;

use crate::{
    client::ClientContext,
    controls::cycle_table::CycleTable,
    events::{GuiEvent, GuiEventQueue},
};
use anyhow::{anyhow, Result};
use arduinox86_client::{CpuPin, ProgramState, ServerFlags, ServerStatus};
use egui::{
    containers::menu::{MenuButton, MenuConfig},
    PopupCloseBehavior,
};
use egui_notify::Toasts;

pub struct ClientWindow {
    icon_size: f32,
    enable_cycle_logging: bool,
    use_sdram_backend: bool,
    use_smm: bool,
    resolve_bus_stepping: bool,
    ale_interrupt_enabled: bool,
    halt_after_jump: bool,
    debug_enabled: bool,
    last_status_time: Option<Instant>,
    last_cycle_ct: u64,
    last_program_state: ProgramState,
    server_status: Option<ServerStatus>,
    effective_mhz: f32,
    cycle_table: CycleTable,
    have_current_cycles: bool,
}

impl Default for ClientWindow {
    fn default() -> Self {
        Self {
            icon_size: 24.0,
            enable_cycle_logging: false,
            use_sdram_backend: false,
            use_smm: false,
            resolve_bus_stepping: false,
            ale_interrupt_enabled: false,
            halt_after_jump: false,
            debug_enabled: false,
            last_status_time: None,
            last_cycle_ct: 0,
            last_program_state: ProgramState::default(),
            server_status: None,
            effective_mhz: 0.0,
            cycle_table: Default::default(),
            have_current_cycles: false,
        }
    }
}

impl ClientWindow {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }

    pub fn init(&mut self, c_ctx: &ClientContext) {
        // Initialize the window with the current flags from the client context
        self.sync_flags(c_ctx);
        self.cycle_table.set_arch(c_ctx.cpu_type);
    }

    pub fn sync_flags(&mut self, c_ctx: &ClientContext) {
        let flags = c_ctx.cached_flags();

        self.enable_cycle_logging = flags & ServerFlags::ENABLE_CYCLE_LOGGING != 0;
        self.use_sdram_backend = flags & ServerFlags::USE_SDRAM_BACKEND != 0;
        self.debug_enabled = flags & ServerFlags::ENABLE_DEBUG != 0;
        self.use_smm = flags & ServerFlags::USE_SMM != 0;
        self.halt_after_jump = flags & ServerFlags::HALT_AFTER_JUMP != 0;
    }

    pub fn set_server_status(&mut self, c_ctx: &mut ClientContext, server_status: ServerStatus) {
        let update_time = Instant::now();

        if let Some(last_update) = self.last_status_time {
            // Calculate the effective MHz based on the time since the last update
            let elapsed_secs = update_time.duration_since(last_update).as_secs_f32();
            if elapsed_secs > 0.0 {
                self.effective_mhz =
                    (server_status.cycle_ct.saturating_sub(self.last_cycle_ct)) as f32 / elapsed_secs / 1_000_000.0;
            }
            else {
                self.effective_mhz = 0.0; // Avoid division by zero
            }
        }
        else {
            self.effective_mhz = 0.0; // First update, no previous time to compare
        }

        if server_status.state != self.last_program_state {
            log::debug!("Server state changed to: {:?}", server_status);
            self.change_state(c_ctx, server_status.state);
        }

        if let Some(existing_status) = &self.server_status {
            self.last_program_state = existing_status.state;
            self.last_cycle_ct = existing_status.cycle_ct;
        }

        self.server_status = Some(server_status);
        self.last_status_time = Some(update_time);
    }

    pub fn change_state(&mut self, c_ctx: &mut ClientContext, new_state: ProgramState) {
        match new_state {
            ProgramState::StoreDone | ProgramState::StoreDoneSmm => {
                // Get the cycle states from the server.
                if let Ok(cycles) = c_ctx.client.get_cycle_states() {
                    self.cycle_table.set_cycles(cycles);
                    self.have_current_cycles = true;
                }
                else {
                    log::error!("Failed to retrieve cycles from server.");
                }
            }
            _ => {}
        }
    }

    pub fn reset_state(&mut self) {
        self.last_program_state = ProgramState::default();
        self.server_status = None;
        self.cycle_table.clear();
    }

    pub fn push_cycle(&mut self, c_ctx: &mut ClientContext, step: bool) -> Result<()> {
        let cycle = c_ctx.client.get_cycle_state(step).map_err(|e| {
            let err_str = format!("Failed to get cycle state: {}", e);
            log::error!("{}", err_str);
            anyhow!(err_str)
        })?;

        self.cycle_table.push_cycle(cycle);

        Ok(())
    }

    pub fn show(
        &mut self,
        e_ctx: &egui::Context,
        c_ctx: &mut ClientContext,
        events: &mut GuiEventQueue,
        toasts: &mut Toasts,
    ) {
        egui::Window::new("Client Connection")
            .default_width(800.0)
            .default_height(600.0)
            .show(e_ctx, |ui| {
                ui.vertical(|ui| {
                    egui::MenuBar::new().ui(ui, |ui| {
                        MenuButton::new("Options")
                            .config(MenuConfig::default().close_behavior(PopupCloseBehavior::CloseOnClickOutside))
                            .ui(ui, |ui| {
                                if ui.checkbox(&mut self.debug_enabled, "Enable Serial Debug").changed() {
                                    match c_ctx.set_flag_state(ServerFlags::ENABLE_DEBUG, self.debug_enabled) {
                                        Ok(true) => {
                                            let toggle_str = "Serial debug enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "Serial debug disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str = format!("Failed to set serial debug state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui
                                    .checkbox(&mut self.enable_cycle_logging, "Enable Cycle Logging")
                                    .changed()
                                {
                                    match c_ctx
                                        .set_flag_state(ServerFlags::ENABLE_CYCLE_LOGGING, self.enable_cycle_logging)
                                    {
                                        Ok(true) => {
                                            let toggle_str = "Cycle logging enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "Cycle logging disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str = format!("Failed to set cycle logging state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui.checkbox(&mut self.use_sdram_backend, "Use SDRAM Backend").changed() {
                                    match c_ctx.set_flag_state(ServerFlags::USE_SDRAM_BACKEND, self.use_sdram_backend) {
                                        Ok(true) => {
                                            let toggle_str = "SDRAM backend enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "SDRAM backend disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str = format!("Failed to set SDRAM backend: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui.checkbox(&mut self.use_smm, "Use SMM").changed() {
                                    match c_ctx.set_flag_state(ServerFlags::USE_SMM, self.use_smm) {
                                        Ok(true) => {
                                            let toggle_str = "SMM enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "SMM disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str = format!("Failed to set SMM control state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui
                                    .checkbox(&mut self.resolve_bus_stepping, "Resolve bus when stepping")
                                    .changed()
                                {
                                    match c_ctx.set_flag_state(ServerFlags::RESOLVE_BUS_STEP, self.resolve_bus_stepping)
                                    {
                                        Ok(true) => {
                                            let toggle_str = "Bus resolution enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "Bus resolution disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str =
                                                format!("Failed to set bus resolution control state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui.checkbox(&mut self.halt_after_jump, "HALT after jump").changed() {
                                    match c_ctx.set_flag_state(ServerFlags::HALT_AFTER_JUMP, self.halt_after_jump) {
                                        Ok(true) => {
                                            let toggle_str = "HALT after jump enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "Halt after jump disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str = format!("Failed to set Halt after jump state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }

                                if ui
                                    .checkbox(&mut self.ale_interrupt_enabled, "ALE interrupt resets READY")
                                    .changed()
                                {
                                    match c_ctx
                                        .set_flag_state(ServerFlags::ENABLE_ALE_INTERRUPT, self.ale_interrupt_enabled)
                                    {
                                        Ok(true) => {
                                            let toggle_str = "ALE interrupt enabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Ok(false) => {
                                            let toggle_str = "ALE interrupt disabled!".to_string();
                                            log::debug!("{}", toggle_str);
                                            toasts.success(toggle_str);
                                        }
                                        Err(e) => {
                                            let toggle_str =
                                                format!("Failed to set ALE interrupt control state: {}", e);
                                            log::error!("{}", toggle_str);
                                            toasts.error(toggle_str);
                                            self.sync_flags(c_ctx);
                                        }
                                    }
                                }
                            });
                    });

                    ui.horizontal(|ui| {
                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::regular::BOX_ARROW_UP))
                                    .size(self.icon_size),
                            )
                            .on_hover_text("Load registers")
                            .clicked()
                        {
                            self.reset_state();
                            events.push(GuiEvent::ResetState);
                            events.push(GuiEvent::LoadRegisters);
                        }

                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::fill::ERASER)).size(self.icon_size),
                            )
                            .on_hover_text("Erase Memory")
                            .clicked()
                        {
                            // Do disconnect
                            events.push(GuiEvent::EraseMemory);
                        }

                        ui.separator();

                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::fill::PLAY_PAUSE))
                                    .size(self.icon_size),
                            )
                            .on_hover_text("Step")
                            .clicked()
                        {
                            // Do step
                            _ = self.push_cycle(c_ctx, true);
                        }

                        if ui
                            .button(egui::RichText::new(format!("{}", egui_phosphor::fill::PLAY)).size(self.icon_size))
                            .on_hover_text("Run Autonomously")
                            .clicked()
                        {
                            self.reset_state();
                            events.push(GuiEvent::ResetState);
                            events.push(GuiEvent::RunProgram);
                        }

                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::fill::PLAY_PAUSE))
                                    .size(self.icon_size),
                            )
                            .on_hover_text("Pause")
                            .clicked()
                        {
                            // Do pause
                        }

                        if ui
                            .button(egui::RichText::new(format!("{}", egui_phosphor::fill::STOP)).size(self.icon_size))
                            .on_hover_text("Stop")
                            .clicked()
                        {
                            match c_ctx.set_flag_state(ServerFlags::EXECUTE_AUTOMATIC, false) {
                                Ok(_) => {
                                    let toggle_str = "Execution stopped!".to_string();
                                    log::debug!("{}", toggle_str);
                                    toasts.success(toggle_str);
                                }
                                Err(e) => {
                                    let toggle_str = format!("Failed to stop execution: {}", e);
                                    log::error!("{}", toggle_str);
                                    toasts.error(toggle_str);
                                    self.sync_flags(c_ctx);
                                }
                            }
                        }

                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::regular::HAND_PALM))
                                    .size(self.icon_size),
                            )
                            .on_hover_text("Toggle Ready")
                            .clicked()
                        {
                            if let Ok(pin) = c_ctx.client.read_pin(CpuPin::READY) {
                                if pin {
                                    log::debug!("READY pin is high, setting low");
                                    c_ctx.client.write_pin(CpuPin::READY, false).ok();
                                }
                                else {
                                    log::debug!("READY pin is low, setting high");
                                    c_ctx.client.write_pin(CpuPin::READY, true).ok();
                                }
                            }
                            else {
                                log::error!("Failed to read READY pin state");
                            }
                            // Do disconnect
                        }

                        if ui
                            .button(
                                egui::RichText::new(format!("{}", egui_phosphor::regular::PLUGS)).size(self.icon_size),
                            )
                            .on_hover_text("Disconnect")
                            .clicked()
                        {
                            self.reset_state();
                            // Do disconnect
                        }
                    });

                    ui.separator();
                    ui.label(format!(
                        "Connected to {} CPU on port {}",
                        c_ctx.cpu_type.to_string(),
                        c_ctx.port_name
                    ));
                    ui.separator();

                    ui.horizontal(|ui| {
                        ui.label("Server state:");
                        if let Some(server_status) = &self.server_status {
                            ui.label(format!("{:?}", server_status.state));
                        }
                        else {
                            ui.label("Uninitialized");
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Cycle count:");
                        if let Some(server_status) = &self.server_status {
                            ui.label(server_status.cycle_ct.to_string());
                        }
                        else {
                            ui.label("0");
                        }

                        ui.separator();
                        ui.label(format!("Effective MHz: {:.2}", self.effective_mhz));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Address latch:");
                        if let Some(server_status) = &self.server_status {
                            let address_digits = c_ctx.cpu_type.bus_chr_width();
                            ui.label(format!("{:0address_digits$X}", server_status.address_latch));
                        }
                        else {
                            ui.label("0");
                        }
                    });
                });

                ui.separator();
                if let Some(response) = self.cycle_table.show(ui, events) {
                    if response.changed() {
                        if let Ok(value) = u16::from_str_radix(self.cycle_table.data_bus_str(), 16) {
                            match c_ctx.client.write_data_bus(value) {
                                Ok(_) => {
                                    log::debug!("Wrote data bus value: {:04X}", value);
                                }
                                Err(e) => {
                                    let err_str = format!("Failed to write data bus value {:04X}: {}", value, e);
                                    log::error!("{}", err_str);
                                    toasts.error(err_str);
                                }
                            }
                        }
                    }
                }
            });
    }
}
