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
    events::GuiEventQueue,
};
use arduinox86_client::{RegisterSetType, RemoteCpuRegisters};

#[derive(Default)]
pub struct RegisterWindow {
    open: bool,
    pub(crate) reg_type: RegisterSetType,

    pub(crate) control_v2: RegisterControlV2,
    pub(crate) control_v3: RegisterControlV3,
}

impl RegisterWindow {
    pub fn new(reg_type: RegisterSetType) -> Self {
        Self {
            open: false,
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
            _ => {
                unimplemented!("Unsupported register type for getting.");
            }
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

    pub fn show(&mut self, e_ctx: &egui::Context, state_type: CpuStateType, events: &mut GuiEventQueue) {
        if self.open {
            egui::Window::new(format!("{} Registers", state_type))
                .default_width(400.0)
                .default_height(300.0)
                .show(e_ctx, |ui| match self.reg_type {
                    RegisterSetType::Intel286 => self.control_v2.show(ui, events),
                    RegisterSetType::Intel386 | RegisterSetType::Intel386Smm => self.control_v3.show(ui, events),
                    _ => {
                        ui.label("Unsupported register type for display.");
                    }
                });
        }
    }
}
