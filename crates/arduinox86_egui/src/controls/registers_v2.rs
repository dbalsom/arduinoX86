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
use crate::{controls::descriptors_v2::DescriptorControlV2, events::GuiEventQueue, register_state::RegisterString};
use arduinox86_client::{trap::TRAP_FLAG, RemoteCpuRegistersV2};
use egui::Color32;

const REGISTER_LABELS: [&str; 16] = [
    "AX", "SP", "BX", "BP", "CX", "SI", "DX", "DI", "DS", "ES", "SS", "CS", "IP", "FLAGS", "TR", "LDTR",
];
const FLAGS_INDEX: usize = 13;
const LABEL_WIDTH: f32 = 48.0;
const ENTRY_WIDTH: f32 = 64.0;
const FLAG_BITS: [(&str, &str, u16); 10] = [
    ("O", "Overflow", 0x0800),
    ("D", "Direction", 0x0400),
    ("I", "Interrupt enable", 0x0200),
    ("T", "Trap", TRAP_FLAG),
    ("S", "Sign", 0x0080),
    ("Z", "Zero", 0x0040),
    ("A", "Auxiliary carry", 0x0010),
    ("P", "Parity", 0x0004),
    ("C", "Carry", 0x0001),
    ("NT", "Nested task", 0x4000),
];
const MSW_BITS: [(&str, &str, u16); 4] = [
    ("PE", "Protection enable", 0x0001),
    ("MP", "Monitor coprocessor", 0x0002),
    ("EM", "Emulate coprocessor", 0x0004),
    ("TS", "Task switched", 0x0008),
];

pub struct RegisterControlV2 {
    regs: RemoteCpuRegistersV2,
    final_regs: Option<RemoteCpuRegistersV2>,
    reg_strings: [RegisterString; 16],
    descriptors: DescriptorControlV2,
}

impl Default for RegisterControlV2 {
    fn default() -> Self {
        let mut regs = RemoteCpuRegistersV2::default();
        regs.normalize_descriptors();
        // The descriptor controls infer Simulate RESET from this initial CS base.
        regs.cs_desc.set_base_address(0xFF0000);
        let mut descriptors = DescriptorControlV2::default();
        descriptors.set_regs(&regs, None);
        Self {
            reg_strings: Self::strings(&regs, None),
            regs,
            final_regs: None,
            descriptors,
        }
    }
}

impl RegisterControlV2 {
    fn values(regs: &RemoteCpuRegistersV2) -> [u16; 16] {
        [
            regs.ax, regs.sp, regs.bx, regs.bp, regs.cx, regs.si, regs.dx, regs.di, regs.ds, regs.es, regs.ss, regs.cs,
            regs.ip, regs.flags, regs.tr, regs.ldt,
        ]
    }

    fn values_mut(regs: &mut RemoteCpuRegistersV2) -> [&mut u16; 16] {
        [
            &mut regs.ax,
            &mut regs.sp,
            &mut regs.bx,
            &mut regs.bp,
            &mut regs.cx,
            &mut regs.si,
            &mut regs.dx,
            &mut regs.di,
            &mut regs.ds,
            &mut regs.es,
            &mut regs.ss,
            &mut regs.cs,
            &mut regs.ip,
            &mut regs.flags,
            &mut regs.tr,
            &mut regs.ldt,
        ]
    }

    fn strings(initial: &RemoteCpuRegistersV2, final_regs: Option<&RemoteCpuRegistersV2>) -> [RegisterString; 16] {
        let initial_values = Self::values(initial);
        let values = Self::values(final_regs.unwrap_or(initial));
        std::array::from_fn(|i| RegisterString::from_diff(format!("{:04X}", values[i]), values[i] != initial_values[i]))
    }

    pub fn set_regs(&mut self, initial: &RemoteCpuRegistersV2, final_regs: Option<&RemoteCpuRegistersV2>) {
        self.regs = initial.clone();
        if final_regs.is_none() {
            self.regs.flags = Self::normalize_flags(self.regs.flags);
        }
        self.reg_strings = Self::strings(&self.regs, final_regs);
        self.final_regs = final_regs.cloned();
        self.descriptors.set_regs(&self.regs, final_regs);
    }

    pub fn regs(&self) -> &RemoteCpuRegistersV2 {
        &self.regs
    }

    pub fn set_trap_enabled(&mut self, enabled: bool) {
        if self.final_regs.is_none() {
            if (self.regs.flags & TRAP_FLAG != 0) != enabled {
                self.set_flag(TRAP_FLAG, enabled);
            }
            self.descriptors.set_trap_enabled(enabled, &mut self.regs);
        }
    }

    // Commit valid edits immediately so clicking Load/Run uses the visible value.
    // On leaving an invalid field, restore its previous value.
    fn apply_edit(text: &mut String, value: &mut u16, lost_focus: bool, is_flags: bool) {
        if let Ok(parsed) = u16::from_str_radix(text, 16) {
            *value = if is_flags {
                (Self::normalize_flags(parsed) & !TRAP_FLAG) | (*value & TRAP_FLAG)
            }
            else {
                parsed
            };
        }
        if lost_focus || (is_flags && u16::from_str_radix(text, 16).is_ok_and(|parsed| (parsed ^ *value) & TRAP_FLAG != 0)) {
            *text = format!("{:04X}", *value);
        }
    }

    fn normalize_flags(flags: u16) -> u16 {
        (flags | RemoteCpuRegistersV2::FLAGS_RESERVED_SET) & RemoteCpuRegistersV2::FLAGS_RESERVED_MASK
    }

    fn show_register_grid(&mut self, ui: &mut egui::Ui, id: &str, rows: &[(usize, Option<usize>)]) {
        let read_only = self.final_regs.is_some();
        let mut values = Self::values_mut(&mut self.regs);
        let row_height = ui.spacing().interact_size.y;
        egui::Grid::new(id)
            .striped(true)
            .num_columns(4)
            .min_col_width(LABEL_WIDTH)
            .show(ui, |ui| {
                for &(left, right) in rows {
                    for index in [Some(left), right] {
                        let Some(i) = index
                        else {
                            ui.allocate_exact_size(egui::vec2(LABEL_WIDTH, row_height), egui::Sense::hover());
                            ui.allocate_exact_size(egui::vec2(ENTRY_WIDTH, row_height), egui::Sense::hover());
                            continue;
                        };
                        let string = &mut self.reg_strings[i];
                        ui.monospace(REGISTER_LABELS[i]);
                        if read_only {
                            ui.add_sized(
                                [ENTRY_WIDTH, row_height],
                                egui::TextEdit::singleline(&mut string.text.as_str())
                                    .font(egui::TextStyle::Monospace)
                                    .text_color(string.color32),
                            );
                        }
                        else {
                            let response = ui.add_sized(
                                [ENTRY_WIDTH, row_height],
                                egui::TextEdit::singleline(&mut string.text)
                                    .char_limit(4)
                                    .font(egui::TextStyle::Monospace),
                            );
                            if response.changed() || response.lost_focus() {
                                Self::apply_edit(&mut string.text, values[i], response.lost_focus(), i == FLAGS_INDEX);
                            }
                        }
                    }
                    ui.end_row();
                }
            });
    }

    pub fn show(&mut self, ui: &mut egui::Ui, _events: &mut GuiEventQueue) {
        self.show_register_grid(
            ui,
            "registers_286_general",
            &[(0, Some(1)), (2, Some(3)), (4, Some(5)), (6, Some(7))],
        );
        ui.separator();
        // DS, ES, SS, then CS:IP on the final row, matching the 386 register window.
        self.show_register_grid(
            ui,
            "registers_286_segments",
            &[(8, None), (9, None), (10, None), (11, Some(12))],
        );
        ui.separator();
        self.show_register_grid(ui, "registers_286_control", &[(13, None), (14, Some(15))]);
        ui.separator();
        ui.horizontal_wrapped(|ui| {
            for (label, tooltip, mask) in FLAG_BITS {
                if let Some(final_regs) = &self.final_regs {
                    let value = u8::from(final_regs.flags & mask != 0);
                    let changed = (self.regs.flags ^ final_regs.flags) & mask != 0;
                    ui.colored_label(
                        if changed { Color32::CYAN } else { crate::TEXT_COLOR },
                        egui::RichText::new(format!("{label}:{value}")).monospace(),
                    )
                    .on_hover_text(tooltip);
                }
                else {
                    let mut set = self.regs.flags & mask != 0;
                    if ui.add_enabled(mask != TRAP_FLAG, egui::Checkbox::new(&mut set, label))
                        .on_hover_text(tooltip)
                        .on_disabled_hover_text("TF is controlled by Execution mode.")
                        .changed() {
                        self.set_flag(mask, set);
                    }
                }
            }
            if let Some(final_regs) = &self.final_regs {
                let changed = (self.regs.flags ^ final_regs.flags) & 0x3000 != 0;
                ui.colored_label(
                    if changed { Color32::CYAN } else { crate::TEXT_COLOR },
                    egui::RichText::new(format!("IOPL:{}", (final_regs.flags >> 12) & 3)).monospace(),
                );
            }
            else {
                ui.label("IOPL");
                let mut iopl = (self.regs.flags >> 12) & 3;
                if ui.add(egui::DragValue::new(&mut iopl).range(0..=3)).changed() {
                    self.regs.flags = Self::normalize_flags((self.regs.flags & !0x3000) | (iopl << 12));
                    self.reg_strings[FLAGS_INDEX].text = format!("{:04X}", self.regs.flags);
                }
            }
        });
        ui.separator();
        ui.horizontal(|ui| {
            ui.monospace("MSW");
            for (label, tooltip, mask) in MSW_BITS {
                let read_only = self.final_regs.is_some();
                let msw = self.final_regs.as_ref().unwrap_or(&self.regs).msw;
                let mut set = msw & mask != 0;
                let changed = read_only && (self.regs.msw ^ msw) & mask != 0;
                let label = egui::RichText::new(label)
                    .color(if changed { Color32::CYAN } else { crate::TEXT_COLOR });
                if ui
                    .add_enabled(!read_only, egui::Checkbox::new(&mut set, label))
                    .on_hover_text(tooltip)
                    .on_disabled_hover_text(tooltip)
                    .changed()
                {
                    if set {
                        self.regs.msw |= mask;
                    }
                    else {
                        self.regs.msw &= !mask;
                    }
                }
            }
        });
        ui.separator();
        self.descriptors.show(ui, &mut self.regs, self.final_regs.as_ref());
        self.descriptors
            .show_temporaries(ui, &mut self.regs, self.final_regs.as_ref());
    }

    fn set_flag(&mut self, mask: u16, set: bool) {
        if set {
            self.regs.flags |= mask;
        }
        else {
            self.regs.flags &= !mask;
        }
        self.regs.flags = Self::normalize_flags(self.regs.flags);
        self.reg_strings[FLAGS_INDEX].text = format!("{:04X}", self.regs.flags);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn execution_mode_owns_trap_flag() {
        let mut control = RegisterControlV2::default();
        control.set_regs(&RemoteCpuRegistersV2 { flags: 0x0103, ..Default::default() }, None);
        control.set_trap_enabled(false);
        assert_eq!(control.regs().flags, 0x0003);
        let mut text = "FFFF".to_string();
        RegisterControlV2::apply_edit(&mut text, &mut control.regs.flags, false, true);
        assert_eq!(control.regs.flags & TRAP_FLAG, 0);
        assert_eq!(text, "7ED7");
        control.set_trap_enabled(true);
        text = "0000".into();
        RegisterControlV2::apply_edit(&mut text, &mut control.regs.flags, false, true);
        assert_eq!(control.regs.flags, 0x0102);
        assert_eq!(text, "0102");
        let final_regs = RemoteCpuRegistersV2 { flags: 0x0002, ..Default::default() };
        control.set_regs(&control.regs.clone(), Some(&final_regs));
        control.set_trap_enabled(true);
        assert_eq!(control.reg_strings[FLAGS_INDEX].text, "0002");
    }

    #[test]
    fn validates_register_edits() {
        let mut value = 0x1234;
        let mut text = "ABCD".to_string();
        RegisterControlV2::apply_edit(&mut text, &mut value, false, false);
        assert_eq!(value, 0xABCD);
        for invalid in ["10000", "xyz", ""] {
            text = invalid.to_string();
            RegisterControlV2::apply_edit(&mut text, &mut value, true, false);
            assert_eq!(value, 0xABCD);
            assert_eq!(text, "ABCD");
        }
    }

    #[test]
    fn flag_edits_preserve_reserved_bits() {
        for (input, expected) in [("0000", 0x0002), ("FFFF", 0x7ED7), ("8028", 0x0002), ("0004", 0x0006)] {
            let mut text = input.to_string();
            let mut flags = 0x0002;
            RegisterControlV2::apply_edit(&mut text, &mut flags, false, true);
            assert_eq!(flags, expected, "Reserved bits must be valid even while editing");
            RegisterControlV2::apply_edit(&mut text, &mut flags, true, true);
            assert_eq!(text, format!("{expected:04X}"));
        }
    }

    #[test]
    fn normalizes_only_initial_flags() {
        let initial = RemoteCpuRegistersV2 {
            flags: 0xFFFF,
            ..Default::default()
        };
        let mut control = RegisterControlV2::default();
        control.set_regs(&initial, None);
        assert_eq!(control.regs().flags, 0x7FD7);
        assert_eq!(control.reg_strings[FLAGS_INDEX].text, "7FD7");
        let final_regs = RemoteCpuRegistersV2 {
            flags: 0x8000,
            ..Default::default()
        };
        control.set_regs(&initial, Some(&final_regs));
        assert_eq!(control.reg_strings[FLAGS_INDEX].text, "8000");
    }

    #[test]
    fn flag_edits_update_cpu_register() {
        let mut control = RegisterControlV2::default();
        control.set_flag(0x0001, true);
        control.set_flag(0x0400, true);
        assert_eq!(control.regs().flags, 0x0403);
        assert_eq!(control.reg_strings[FLAGS_INDEX].text, "0403");
        control.set_flag(0x0001, false);
        assert_eq!(control.regs().flags, 0x0402);
    }

    #[test]
    fn highlights_and_clears_final_changes() {
        let initial = RemoteCpuRegistersV2::default();
        let mut final_regs = initial.clone();
        final_regs.ax = 0xBEEF;
        final_regs.ip = 0x1234;
        final_regs.flags = 0x0043;
        let mut control = RegisterControlV2::default();
        control.set_regs(&initial, Some(&final_regs));
        assert_eq!(control.reg_strings[0].text, "BEEF");
        assert_eq!(control.reg_strings[0].color32, Color32::CYAN);
        assert_eq!(control.reg_strings[1].color32, crate::TEXT_COLOR);
        assert_eq!(control.reg_strings[12].text, "1234");
        assert_eq!(control.reg_strings[FLAGS_INDEX].text, "0043");
        assert_eq!(control.regs().ax, initial.ax);
        control.set_regs(&initial, None);
        assert!(control.final_regs.is_none());
        assert_eq!(control.reg_strings[0].text, "0000");
        assert_eq!(control.reg_strings[0].color32, crate::TEXT_COLOR);
    }

    #[test]
    fn compares_step_and_final_snapshots() {
        use crate::{enums::CpuStateType, windows::RegisterWindow};
        use arduinox86_client::{trap::{InstructionState, TrapStatus}, RemoteCpuRegisters};
        let initial = RemoteCpuRegistersV2 { flags: 0x0002, ..Default::default() };
        let initial_regs = RemoteCpuRegisters::V2(initial.clone());
        let mut first = initial.clone();
        first.ax = 1;
        first.flags |= 0x0001;
        first.ds_desc.set_base_address(0x12345);
        let mut wire = [0; InstructionState::WIRE_SIZE];
        wire[0] = 1;
        let mut state = InstructionState::from_bytes(&wire).unwrap();
        state.status = TrapStatus::Paused;
        state.run_id = 1;
        state.sequence = 1;
        state.capture_cycle = 10;
        let mut window = RegisterWindow::default();
        window.loaded(&initial_regs, true);
        assert_eq!(window.readout_state_type(), CpuStateType::Current);
        assert!(window.control_v2.final_regs.is_some(), "current registers must be read-only");
        window.set_readout(&initial_regs, &RemoteCpuRegisters::V2(first.clone()), Some(&state));
        assert_eq!(window.control_v2.reg_strings[0].color32, Color32::CYAN);
        assert_eq!(window.control_v2.reg_strings[FLAGS_INDEX].color32, Color32::CYAN);

        let mut second = first.clone();
        second.bx = 2;
        state.sequence += 1;
        state.capture_cycle += 10;
        let second_regs = RemoteCpuRegisters::V2(second);
        // Repeated delivery of the same snapshot must not erase its highlights.
        for _ in 0..2 {
            window.set_readout(&initial_regs, &second_regs, Some(&state));
            assert_eq!(window.control_v2.reg_strings[0].color32, crate::TEXT_COLOR);
            assert_eq!(window.control_v2.reg_strings[2].color32, Color32::CYAN);
            assert_eq!(window.control_v2.reg_strings[FLAGS_INDEX].color32, crate::TEXT_COLOR);
            assert_eq!(window.control_v2.regs().ds_desc.base_address(), first.ds_desc.base_address());
        }
        // The endpoint can finalize the very same paused image without a clock.
        state.status = TrapStatus::Captured;
        window.set_readout(&initial_regs, &second_regs, Some(&state));
        assert_eq!(window.readout_state_type(), CpuStateType::Final);
        assert_eq!(window.control_v2.reg_strings[0].color32, Color32::CYAN);
        assert_eq!(window.control_v2.reg_strings[2].color32, Color32::CYAN);
        assert_eq!(window.control_v2.reg_strings[FLAGS_INDEX].color32, Color32::CYAN);
        assert_eq!(window.control_v2.regs().ds_desc.base_address(), initial.ds_desc.base_address());
    }
}
