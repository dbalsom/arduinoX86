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
use crate::register_state::RegisterString;
use arduinox86_client::{RemoteCpuRegistersV2, SegmentDescriptorV1};

const DESCRIPTOR_LABELS: [&str; 8] = ["ES", "CS", "SS", "DS", "GDT", "LDT", "IDT", "TSS"];
const FIELD_DIGITS: [usize; 3] = [6, 4, 2];
const TEMPORARY_OFFSETS: [u16; 10] = [0x800, 0x802, 0x804, 0x808, 0x80A, 0x80C, 0x80E, 0x810, 0x812, 0x814];

#[derive(Default)]
pub(super) struct DescriptorControlV2 {
    strings: [[RegisterString; 3]; 8],
    temporary_strings: [RegisterString; 10],
    link_segment_bases: bool,
    simulate_reset: bool,
    // Apply once per new result, so the user can subsequently collapse the section.
    pending_open: Option<bool>,
    pending_temporaries_open: Option<bool>,
}

impl DescriptorControlV2 {
    fn descriptors(regs: &RemoteCpuRegistersV2) -> [SegmentDescriptorV1; 8] {
        [
            regs.es_desc,
            regs.cs_desc,
            regs.ss_desc,
            regs.ds_desc,
            regs.gdt_desc,
            regs.ldt_desc,
            regs.idt_desc,
            regs.tss_desc,
        ]
    }

    fn descriptors_mut(regs: &mut RemoteCpuRegistersV2) -> [&mut SegmentDescriptorV1; 8] {
        [
            &mut regs.es_desc,
            &mut regs.cs_desc,
            &mut regs.ss_desc,
            &mut regs.ds_desc,
            &mut regs.gdt_desc,
            &mut regs.ldt_desc,
            &mut regs.idt_desc,
            &mut regs.tss_desc,
        ]
    }

    fn fields(desc: SegmentDescriptorV1) -> [u32; 3] {
        [
            desc.base_address(),
            u32::from(desc.limit()),
            u32::from(desc.into_bytes()[3]),
        ]
    }

    fn set_fields(desc: &mut SegmentDescriptorV1, fields: [u32; 3]) {
        desc.set_base_address(fields[0]);
        desc.set_limit(fields[1] as u16);
        let mut bytes = desc.into_bytes();
        bytes[3] = fields[2] as u8;
        *desc = SegmentDescriptorV1::from_bytes(bytes);
    }

    fn temporaries(regs: &RemoteCpuRegistersV2) -> [u16; 10] {
        [
            regs.x0, regs.x1, regs.x2, regs.x3, regs.x4, regs.x5, regs.x6, regs.x7, regs.x8, regs.x9,
        ]
    }

    fn temporaries_mut(regs: &mut RemoteCpuRegistersV2) -> [&mut u16; 10] {
        [
            &mut regs.x0,
            &mut regs.x1,
            &mut regs.x2,
            &mut regs.x3,
            &mut regs.x4,
            &mut regs.x5,
            &mut regs.x6,
            &mut regs.x7,
            &mut regs.x8,
            &mut regs.x9,
        ]
    }

    fn segment_bases(&self, regs: &RemoteCpuRegistersV2) -> [u32; 4] {
        let mut bases = [regs.es, regs.cs, regs.ss, regs.ds].map(|selector| u32::from(selector) << 4);
        if self.simulate_reset && regs.cs == 0xF000 {
            bases[1] = 0xFF0000;
        }
        bases
    }

    pub(super) fn set_regs(&mut self, initial: &RemoteCpuRegistersV2, final_regs: Option<&RemoteCpuRegistersV2>) {
        let initial_descs = Self::descriptors(initial).map(Self::fields);
        let descs = Self::descriptors(final_regs.unwrap_or(initial)).map(Self::fields);
        self.strings = std::array::from_fn(|row| {
            std::array::from_fn(|col| {
                RegisterString::from_diff(
                    format!("{:0width$X}", descs[row][col], width = FIELD_DIGITS[col]),
                    descs[row][col] != initial_descs[row][col],
                )
            })
        });
        let initial_temporaries = Self::temporaries(initial);
        let temporaries = Self::temporaries(final_regs.unwrap_or(initial));
        self.temporary_strings = std::array::from_fn(|i| {
            RegisterString::from_diff(
                format!("{:04X}", temporaries[i]),
                temporaries[i] != initial_temporaries[i],
            )
        });
        self.simulate_reset = initial.cs == 0xF000 && initial.cs_desc.base_address() == 0xFF0000;
        self.link_segment_bases = self
            .segment_bases(initial)
            .iter()
            .enumerate()
            .all(|(i, base)| *base == initial_descs[i][0]);
        let temporaries_changed = temporaries != initial_temporaries;
        self.pending_open = (descs != initial_descs).then_some(true);
        self.pending_temporaries_open = temporaries_changed.then_some(true);
    }

    // Keep only bases linked: custom access bytes and limits must survive selector edits.
    pub(super) fn sync_segment_bases(&mut self, regs: &mut RemoteCpuRegistersV2) {
        if self.link_segment_bases {
            let bases = self.segment_bases(regs);
            for (i, desc) in Self::descriptors_mut(regs).into_iter().take(4).enumerate() {
                desc.set_base_address(bases[i]);
                self.strings[i][0].text = format!("{:06X}", bases[i]);
            }
        }
    }

    fn apply_edit(text: &mut String, value: &mut u32, digits: usize, lost_focus: bool) {
        if let Ok(parsed) = u32::from_str_radix(text, 16) {
            if parsed < (1 << (digits * 4)) {
                *value = parsed;
            }
        }
        if lost_focus {
            *text = format!("{:0digits$X}", *value);
        }
    }

    fn hex_field(ui: &mut egui::Ui, string: &mut RegisterString, value: &mut u32, digits: usize, read_only: bool) {
        let size = [if digits == 6 { 80.0 } else { 64.0 }, ui.spacing().interact_size.y];
        if read_only {
            ui.add_sized(
                size,
                egui::TextEdit::singleline(&mut string.text.as_str())
                    .font(egui::TextStyle::Monospace)
                    .text_color(string.color32),
            );
        }
        else {
            let response = ui.add_sized(
                size,
                egui::TextEdit::singleline(&mut string.text)
                    .char_limit(digits)
                    .font(egui::TextStyle::Monospace),
            );
            if response.changed() || response.lost_focus() {
                Self::apply_edit(&mut string.text, value, digits, response.lost_focus());
            }
        }
    }

    pub(super) fn show(
        &mut self,
        ui: &mut egui::Ui,
        regs: &mut RemoteCpuRegistersV2,
        final_regs: Option<&RemoteCpuRegistersV2>,
    ) -> egui::collapsing_header::CollapsingResponse<()> {
        let read_only = final_regs.is_some();
        // Sync even when collapsed, since selectors are edited above this section.
        if !read_only {
            self.sync_segment_bases(regs);
        }
        egui::CollapsingHeader::new("Descriptors")
            .id_salt("descriptors_286")
            .open(self.pending_open.take())
            .show(ui, |ui| {
                if !read_only {
                    if ui
                        .checkbox(&mut self.link_segment_bases, "Link segment bases to selectors")
                        .on_hover_text(
                            "Use selector × 16 for ES, CS, SS, and DS. Uncheck to edit their bases independently.",
                        )
                        .changed()
                    {
                        self.sync_segment_bases(regs);
                    }
                    if ui
                        .add_enabled(
                            self.link_segment_bases,
                            egui::Checkbox::new(&mut self.simulate_reset, "Simulate RESET"),
                        )
                        .on_hover_text("When linked, CS = F000 uses base FF0000 instead of 0F0000.")
                        .changed()
                    {
                        self.sync_segment_bases(regs);
                    }
                }
                egui::Grid::new("descriptor_file_286")
                    .striped(true)
                    .num_columns(4)
                    .min_col_width(48.0)
                    .show(ui, |ui| {
                        ui.label("Register");
                        ui.label("Base");
                        ui.label("Limit");
                        ui.label("Access")
                            .on_hover_text("Raw access byte: type [3:0], S [4], DPL [6:5], P [7].");
                        ui.end_row();
                        for (row, desc) in Self::descriptors_mut(regs).into_iter().enumerate() {
                            ui.monospace(DESCRIPTOR_LABELS[row]);
                            let mut fields = Self::fields(*desc);
                            for col in 0..3 {
                                let linked = !read_only && self.link_segment_bases && row < 4 && col == 0;
                                let unused_access = !read_only && matches!(row, 4 | 6) && col == 2;
                                let response = ui.add_enabled_ui(!linked && !unused_access, |ui| {
                                    Self::hex_field(
                                        ui,
                                        &mut self.strings[row][col],
                                        &mut fields[col],
                                        FIELD_DIGITS[col],
                                        read_only,
                                    );
                                });
                                if unused_access {
                                    response
                                        .response
                                        .on_disabled_hover_text("Unused access-byte slot for GDTR/IDTR.");
                                }
                            }
                            if !read_only {
                                Self::set_fields(desc, fields);
                            }
                            ui.end_row();
                        }
                    });
            })
    }

    pub(super) fn show_temporaries(
        &mut self,
        ui: &mut egui::Ui,
        regs: &mut RemoteCpuRegistersV2,
        final_regs: Option<&RemoteCpuRegistersV2>,
    ) -> egui::collapsing_header::CollapsingResponse<()> {
        let read_only = final_regs.is_some();
        egui::CollapsingHeader::new("Temporaries")
            .id_salt("temporaries_286")
            .open(self.pending_temporaries_open.take())
            .show(ui, |ui| {
                egui::Grid::new("temporaries_file_286")
                    .striped(true)
                    .num_columns(4)
                    .show(ui, |ui| {
                        for (i, word) in Self::temporaries_mut(regs).into_iter().enumerate() {
                            ui.monospace(format!("X{i}"))
                                .on_hover_text(format!("LOADALL/STOREALL temporary at {:04X}h", TEMPORARY_OFFSETS[i]));
                            let mut value = u32::from(*word);
                            Self::hex_field(ui, &mut self.temporary_strings[i], &mut value, 4, read_only);
                            if !read_only {
                                *word = value as u16;
                            }
                            if i % 2 == 1 {
                                ui.end_row();
                            }
                        }
                    });
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_edits_accept_full_width_and_reject_overflow() {
        for (digits, maximum) in [(6, 0xFFFFFF), (4, 0xFFFF), (2, 0xFF)] {
            let mut value = 0;
            let mut text = format!("{maximum:X}");
            DescriptorControlV2::apply_edit(&mut text, &mut value, digits, false);
            assert_eq!(value, maximum);
            for invalid in [format!("{:X}", maximum + 1), "xyz".to_string(), String::new()] {
                text = invalid;
                DescriptorControlV2::apply_edit(&mut text, &mut value, digits, true);
                assert_eq!(value, maximum);
                assert_eq!(text, format!("{maximum:X}"));
            }
        }
        for bytes in [[0; 6], [0xFF; 6], [0x56, 0x34, 0x12, 0x9B, 0xCD, 0xAB]] {
            let mut desc = SegmentDescriptorV1::default();
            DescriptorControlV2::set_fields(
                &mut desc,
                DescriptorControlV2::fields(SegmentDescriptorV1::from_bytes(bytes)),
            );
            assert_eq!(desc.into_bytes(), bytes);
        }
    }

    #[test]
    fn linking_tracks_selectors_without_replacing_limits_or_access_bytes() {
        let mut regs = RemoteCpuRegistersV2::default();
        regs.normalize_descriptors();
        regs.cs_desc = SegmentDescriptorV1::from_bytes([0, 0, 0x0F, 0x9B, 0x34, 0x12]);
        let mut control = DescriptorControlV2::default();
        control.set_regs(&regs, None);
        assert!(control.link_segment_bases);
        regs.cs = 0xABCD;
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.cs_desc.into_bytes(), [0xD0, 0xBC, 0x0A, 0x9B, 0x34, 0x12]);
        assert_eq!(control.strings[1][0].text, "0ABCD0");

        regs.cs_desc.set_base_address(0x123456);
        control.set_regs(&regs, None);
        assert!(!control.link_segment_bases);
        regs.cs = 0;
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.cs_desc.base_address(), 0x123456);
    }

    #[test]
    fn simulated_reset_remaps_only_linked_cs_and_uses_the_high_reset_vector() {
        let mut regs = RemoteCpuRegistersV2 {
            es: 0xF000,
            ss: 0xF000,
            ds: 0xF000,
            ..Default::default()
        };
        regs.normalize_descriptors();
        let mut control = DescriptorControlV2::default();
        control.set_regs(&regs, None);
        assert!(!control.simulate_reset);

        control.simulate_reset = true;
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.cs_desc.base_address(), 0xFF0000);
        assert_eq!(regs.calculate_code_address(), 0xFFFFF0);
        assert_eq!(control.strings[1][0].text, "FF0000");
        for desc in [regs.es_desc, regs.ss_desc, regs.ds_desc] {
            assert_eq!(desc.base_address(), 0x0F0000);
        }

        // Restoring this initial state must retain its reset mapping.
        control.set_regs(&regs, None);
        assert!(control.simulate_reset);
        assert!(control.link_segment_bases);

        control.simulate_reset = false;
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.calculate_code_address(), 0x0FFFF0);
        control.simulate_reset = true;
        regs.cs = 0x1234;
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.cs_desc.base_address(), 0x012340);

        // Unlinked descriptors stay editable even if Simulate RESET remains checked.
        control.link_segment_bases = false;
        regs.cs = 0xF000;
        regs.cs_desc.set_base_address(0xABCDEF);
        control.sync_segment_bases(&mut regs);
        assert_eq!(regs.cs_desc.base_address(), 0xABCDEF);
    }

    #[test]
    fn final_changes_highlight_individual_fields_and_request_expansion() {
        let initial = RemoteCpuRegistersV2::default();
        let mut control = DescriptorControlV2::default();
        control.set_regs(&initial, Some(&initial));
        assert_eq!(control.pending_open, None);
        for row in 0..8 {
            for col in 0..3 {
                let mut result = initial.clone();
                let desc = &mut DescriptorControlV2::descriptors_mut(&mut result)[row];
                let mut fields = DescriptorControlV2::fields(**desc);
                fields[col] ^= 1;
                DescriptorControlV2::set_fields(desc, fields);
                control.set_regs(&initial, Some(&result));
                assert_eq!(control.pending_open, Some(true));
                for r in 0..8 {
                    for c in 0..3 {
                        assert_eq!(
                            control.strings[r][c].color32,
                            if (r, c) == (row, col) {
                                egui::Color32::CYAN
                            }
                            else {
                                crate::TEXT_COLOR
                            }
                        );
                    }
                }
            }
        }
        for i in 0..10 {
            let mut result = initial.clone();
            *DescriptorControlV2::temporaries_mut(&mut result)[i] ^= 1;
            control.set_regs(&initial, Some(&result));
            assert_eq!(control.pending_open, None);
            assert_eq!(control.pending_temporaries_open, Some(true));
            assert_eq!(control.temporary_strings[i].color32, egui::Color32::CYAN);
        }
    }

    #[test]
    fn final_section_opens_on_changes_and_respects_subsequent_manual_collapse() {
        for temporaries in [false, true] {
            let context = egui::Context::default();
            context.style_mut(|style| style.animation_time = 0.0);
            let initial = RemoteCpuRegistersV2::default();
            let mut result = initial.clone();
            let mut control = DescriptorControlV2::default();
            let mut header_id = egui::Id::NULL;
            let mut render = |control: &mut DescriptorControlV2, result: &RemoteCpuRegistersV2| {
                let _ = context.run(egui::RawInput::default(), |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let descriptors = control.show(ui, &mut initial.clone(), Some(result));
                        let temps = control.show_temporaries(ui, &mut initial.clone(), Some(result));
                        assert_eq!(
                            descriptors.header_response.rect.left(),
                            temps.header_response.rect.left()
                        );
                        assert!(temps.header_response.rect.top() > descriptors.header_response.rect.top());
                        let responses = [descriptors, temps];
                        header_id = responses[usize::from(temporaries)].header_response.id;
                        let other_id = responses[usize::from(!temporaries)].header_response.id;
                        assert!(!egui::collapsing_header::CollapsingState::load(ctx, other_id)
                            .unwrap()
                            .is_open());
                    });
                });
                header_id
            };
            control.set_regs(&initial, Some(&result));
            let id = render(&mut control, &result);
            assert!(!egui::collapsing_header::CollapsingState::load(&context, id)
                .unwrap()
                .is_open());

            if temporaries {
                result.x9 = 0x1234;
            }
            else {
                result.tss_desc.set_base_address(0xABCDEF);
            }
            control.set_regs(&initial, Some(&result));
            render(&mut control, &result);
            let mut state = egui::collapsing_header::CollapsingState::load(&context, id).unwrap();
            assert!(state.is_open());
            assert_eq!(control.pending_open, None);
            assert_eq!(control.pending_temporaries_open, None);
            state.set_open(false);
            state.store(&context);
            render(&mut control, &result);
            assert!(!egui::collapsing_header::CollapsingState::load(&context, id)
                .unwrap()
                .is_open());

            // A new changed readout reopens even a previously collapsed section.
            control.set_regs(&initial, Some(&result));
            render(&mut control, &result);
            assert!(egui::collapsing_header::CollapsingState::load(&context, id)
                .unwrap()
                .is_open());
        }
    }
}
