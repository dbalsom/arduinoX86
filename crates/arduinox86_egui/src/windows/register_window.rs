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
    controls::{registers_v2::RegisterControlV2, registers_v3::RegisterControlV3},
    enums::CpuStateType,
    events::{GuiEvent, GuiEventQueue},
};
use arduinox86_client::{trap::{InstructionState, TrapStatus, TRAP_FLAG}, Registers32, RegisterSetType, RemoteCpuRegisters};

struct TrapRegisterSnapshot {
    key: (u32, u32, u32), // Run, trap sequence, capture cycle.
    regs: RemoteCpuRegisters,
}

#[derive(Default)]
pub struct RegisterWindow {
    open: bool,
    current: bool,
    trap_snapshot: Option<TrapRegisterSnapshot>,
    pub(crate) reg_type: RegisterSetType,

    pub(crate) control_v2: RegisterControlV2,
    pub(crate) control_v3: RegisterControlV3,
}

impl RegisterWindow {
    pub fn new(reg_type: RegisterSetType) -> Self {
        Self {
            open: false,
            current: false,
            trap_snapshot: None,
            reg_type,
            control_v2: RegisterControlV2::default(),
            control_v3: RegisterControlV3::new(),
        }
    }

    pub fn open(&self) -> &bool {
        &self.open
    }

    pub fn open_mut(&mut self) -> &mut bool {
        &mut self.open
    }

    pub fn clear_readout(&mut self) {
        self.open = false;
        self.current = false;
        self.trap_snapshot = None;
    }

    pub fn loaded(&mut self, regs: &RemoteCpuRegisters, trap: bool) {
        self.clear_readout();
        if trap {
            self.set_regs(regs, Some(regs));
            self.current = true;
            self.open = true;
        }
    }

    pub fn observe_instruction(&mut self, state: &InstructionState) {
        if state.status == TrapStatus::Idle
            || self.trap_snapshot.as_ref().is_some_and(|snapshot| snapshot.key.0 != state.run_id) {
            self.clear_readout();
        }
    }

    pub fn readout_state_type(&self) -> CpuStateType {
        if self.current { CpuStateType::Current } else { CpuStateType::Final }
    }

    pub fn set_readout(
        &mut self,
        initial: &RemoteCpuRegisters,
        regs: &RemoteCpuRegisters,
        instruction: Option<&InstructionState>,
    ) {
        if let Some(state) = instruction.filter(|state| matches!(state.status, TrapStatus::Paused | TrapStatus::Captured)) {
            let key = (state.run_id, state.sequence, state.capture_cycle);
            self.current = state.status == TrapStatus::Paused;
            // Duplicate polls must preserve the last step's highlighting. 
            // A final snapshot always switches back to comparison with the initial state.
            if !self.current || self.trap_snapshot.as_ref().map(|snapshot| snapshot.key) != Some(key) {
                let baseline = if self.current {
                    self.trap_snapshot.as_ref()
                        .filter(|snapshot| snapshot.key.0 == state.run_id)
                        .map(|snapshot| &snapshot.regs)
                        .unwrap_or(initial)
                } else {
                    initial
                }.clone();
                self.set_regs(&baseline, Some(regs));
                self.trap_snapshot = Some(TrapRegisterSnapshot { key, regs: regs.clone() });
            }
        } else {
            self.current = false;
            self.trap_snapshot = None;
            self.set_regs(initial, Some(regs));
        }
        self.open = true;
    }

    pub fn set_trap_enabled(&mut self, enabled: bool) {
        self.control_v2.set_trap_enabled(enabled);
        if self.control_v3.final_regs.is_none() {
            let flags = self.control_v3.regs.eflags();
            let tf = if enabled { u32::from(TRAP_FLAG) } else { 0 };
            if flags & u32::from(TRAP_FLAG) != tf {
                self.control_v3.regs.set_eflags((flags & !u32::from(TRAP_FLAG)) | tf);
                self.control_v3.reg_strings.flags.t_fl.text = u8::from(enabled).to_string();
            }
        }
    }

    pub fn set_regs(&mut self, initial_regs: &RemoteCpuRegisters, final_regs: Option<&RemoteCpuRegisters>) {
        self.reg_type = RegisterSetType::from(initial_regs);
        match (initial_regs, final_regs) {
            (RemoteCpuRegisters::V2(initial_regs_v2), Some(RemoteCpuRegisters::V2(final_regs_v2))) => {
                self.control_v2.set_regs(initial_regs_v2, Some(final_regs_v2));
            }
            (RemoteCpuRegisters::V2(initial_regs_v2), None) => {
                self.control_v2.set_regs(initial_regs_v2, None);
            }
            (RemoteCpuRegisters::V3(initial_regs_v3), Some(RemoteCpuRegisters::V3(final_regs_v3))) => {
                self.control_v3.set_regs(initial_regs_v3, Some(final_regs_v3));
            }
            (RemoteCpuRegisters::V3(initial_regs_v3), None) => {
                self.control_v3.set_regs(initial_regs_v3, None);
            }
            _ => {
                log::warn!("Unsupported register type for setting.");
            }
        }
    }

    pub fn regs(&self) -> RemoteCpuRegisters {
        match self.reg_type {
            RegisterSetType::Intel286 => RemoteCpuRegisters::V2(self.control_v2.regs().clone()),
            RegisterSetType::Intel386 | RegisterSetType::Intel386Smm => {
                RemoteCpuRegisters::V3(self.control_v3.regs().clone())
            }
            _ => unimplemented!("Unsupported register type for getting."),
        }
    }

    /// Preserve the 286 LOADALL image, including explicitly edited descriptor caches.
    pub fn load_regs(&self) -> RemoteCpuRegisters {
        let mut regs = self.regs();
        if !matches!(regs, RemoteCpuRegisters::V2(_)) {
            regs.normalize();
        }
        regs
    }

    /// Mount code using the same CS cache and IP that will be loaded into the CPU.
    pub fn code_address(&self) -> u32 {
        self.load_regs().code_address()
    }

    pub fn show(&mut self, e_ctx: &egui::Context, state_type: CpuStateType, events: &mut GuiEventQueue, busy: bool) {
        if self.open {
            egui::Window::new(format!("{} Registers", state_type))
                // Current and Final are the same readout window. 
                // we keep persisted position, size and child IDs when the title changes.
                .id(egui::Id::new(if state_type == CpuStateType::Initial { "Initial Registers" } else { "Final Registers" }))
                .resizable(!matches!(state_type, CpuStateType::Initial))
                .default_width(400.0)
                .default_height(300.0)
                .show(e_ctx, |ui| {
                    if matches!(state_type, CpuStateType::Initial) {
                        if ui
                            .add_enabled(
                                !busy,
                                egui::Button::new(egui::RichText::new(egui_phosphor::regular::BOX_ARROW_UP).size(24.0)),
                            )
                            .on_hover_text("Load registers")
                            .clicked()
                        {
                            events.push(GuiEvent::LoadRegisters);
                        }
                        ui.separator();
                    }
                    match self.reg_type {
                        RegisterSetType::Intel286 => self.control_v2.show(ui, events),
                        RegisterSetType::Intel386 | RegisterSetType::Intel386Smm => self.control_v3.show(ui, events),
                        _ => {
                            ui.label("Unsupported register type for display.");
                        }
                    }
                });
        }
    }
}
