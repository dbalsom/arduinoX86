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

use crate::{DEFAULT_FONT_SIZE, TEXT_COLOR};

use arduinox86_client::{BusState, CpuWidth, DataWidth, ServerCpuType, ServerCycleState, TState};
use egui::{text::LayoutJob, Color32, FontId, Response, TextFormat, TextStyle, Ui, Widget};

pub const ALE_COLOR: Color32 = Color32::from_rgba_premultiplied(0xf9, 0x7a, 0x48, 0xff);
pub const DOT_COLOR: Color32 = Color32::GRAY;
pub const COLON_COLOR: Color32 = Color32::GRAY;
pub const BRACKET_COLOR: Color32 = Color32::GRAY;
//pub const MEM_COLOR: Color32 = Color32::LIGHT_BLUE;
//pub const IO_COLOR: Color32 = Color32::ORANGE;
pub const MEM_COLOR: Color32 = TEXT_COLOR;
pub const IO_COLOR: Color32 = TEXT_COLOR;
pub const PIN_COLOR: Color32 = TEXT_COLOR;
pub const READ_COLOR: Color32 = desat_to_color32(BRIGHT_GREEN, DESAT);
pub const WRITE_COLOR: Color32 = desat_to_color32(BRIGHT_BLUE, DESAT);
pub const BHE_COLOR: Color32 = desat_to_color32(BRIGHT_YELLOW, DESAT);

pub struct CycleDisplay<'a> {
    font_size: f32,
    arch: ServerCpuType,
    state: ServerCycleState,
    address_latch: &'a mut u32,
    data_bus_opt: Option<&'a mut String>,
    edit_dbus: bool,
}

// ---------- base ANSI/VGA-like RGB triplets ----------
const RED: (u8, u8, u8) = (170, 0, 0);
const YELLOW: (u8, u8, u8) = (170, 170, 0);
const BRIGHT_YELLOW: (u8, u8, u8) = (255, 255, 85);
const BRIGHT_MAGENTA: (u8, u8, u8) = (255, 85, 255);
const CYAN: (u8, u8, u8) = (0, 170, 170);
const BRIGHT_BLUE: (u8, u8, u8) = (160, 160, 255);
const BRIGHT_GREEN: (u8, u8, u8) = (85, 255, 85);
const WHITE: (u8, u8, u8) = (255, 255, 255);

// ---------- desaturation helpers (const, no floats) ----------
/// Rec.709 luma in u8 using integer math.
const fn luma709_u8(r: u8, g: u8, b: u8) -> u8 {
    // 0.2126R + 0.7152G + 0.0722B  ≈  (54*R + 183*G + 19*B)/256
    let y = 54u16 * r as u16 + 183u16 * g as u16 + 19u16 * b as u16;
    (y >> 8) as u8
}

/// Linear blend of `c` toward `gray` by t/256 (t in 0..=256).
#[inline]
const fn mix_to_gray_ch(c: u8, gray: u8, t256: u16) -> u8 {
    let t = if t256 > 256 { 256 } else { t256 }; // clamp
    let inv = 256u16 - t;
    (((c as u16) * inv + (gray as u16) * t) >> 8) as u8
}

/// Desaturate RGB toward its luma by t/256.
const fn desaturate_rgb_t256(r: u8, g: u8, b: u8, t256: u16) -> (u8, u8, u8) {
    let gray = luma709_u8(r, g, b);
    (
        mix_to_gray_ch(r, gray, t256),
        mix_to_gray_ch(g, gray, t256),
        mix_to_gray_ch(b, gray, t256),
    )
}

/// Desaturate and produce `Color32` (const-friendly).
const fn desat_to_color32(rgb: (u8, u8, u8), t256: u16) -> Color32 {
    let (r, g, b) = rgb;
    let (r2, g2, b2) = desaturate_rgb_t256(r, g, b, t256);
    Color32::from_rgba_premultiplied(r2, g2, b2, 255)
}

const DESAT: u16 = 160;

pub const BUS_STATE_COLORS: [Color32; 8] = [
    desat_to_color32(RED, DESAT),            // INTA
    desat_to_color32(YELLOW, DESAT),         // IOR
    desat_to_color32(BRIGHT_YELLOW, DESAT),  // IOW
    desat_to_color32(BRIGHT_MAGENTA, DESAT), // HALT
    desat_to_color32(CYAN, DESAT),           // CODE
    desat_to_color32(BRIGHT_BLUE, DESAT),    // MEMR
    desat_to_color32(BRIGHT_GREEN, DESAT),   // MEMW
    desat_to_color32(WHITE, DESAT),          // PASV
];

// pub enum TState {
//     Ti,
//     T1,
//     T2,
//     T3,
//     T4,
//     Tw,
// }

pub const T_STATES: [(&str, Color32); 8] = [
    ("Ti", TEXT_COLOR),
    ("T1", desat_to_color32(BRIGHT_YELLOW, DESAT)),
    ("T2", desat_to_color32(YELLOW, DESAT)),
    ("T3", desat_to_color32(YELLOW, DESAT)),
    ("T4", desat_to_color32(YELLOW, DESAT)),
    ("Tw", desat_to_color32(BRIGHT_BLUE, DESAT)),
    ("T?", desat_to_color32(WHITE, DESAT)),
    ("T?", desat_to_color32(WHITE, DESAT)),
];

impl<'a> CycleDisplay<'a> {
    pub fn new(
        arch: ServerCpuType,
        state: ServerCycleState,
        address_latch: &'a mut u32,
        data_bus_opt: Option<&'a mut String>,
    ) -> Self {
        Self {
            font_size: DEFAULT_FONT_SIZE,
            arch,
            state,
            address_latch,
            edit_dbus: data_bus_opt.is_some(),
            data_bus_opt,
        }
    }

    pub fn with_edit_dbus(mut self, edit: bool) -> Self {
        self.edit_dbus = edit;
        self
    }

    pub fn data_width(&self) -> DataWidth {
        let cpu_width = CpuWidth::from(self.arch);
        match cpu_width {
            CpuWidth::Eight => DataWidth::EightLow,
            CpuWidth::Sixteen => {
                if (*self.address_latch & 1 != 0)
                    && (self.state.bus_command_bits & ServerCycleState::COMMAND_BHE_BIT == 0)
                {
                    DataWidth::EightHigh
                }
                else if self.state.pins & ServerCycleState::PIN_BHE == 0 {
                    DataWidth::Sixteen
                }
                else {
                    DataWidth::EightLow
                }
            }
        }
    }

    pub fn data_bus_str(&self) -> String {
        match self.data_width() {
            DataWidth::Invalid => "----".to_string(),
            DataWidth::Sixteen => format!("{:04X}", self.state.data_bus),
            DataWidth::EightLow => format!("{:>4}", format!("{:02X}", self.state.data_bus as u8)),
            DataWidth::EightHigh => format!("{:<4}", format!("{:02X}", (self.state.data_bus >> 8) as u8)),
        }
    }
}

macro_rules! append_colored {
    ($job:expr, $text:expr, $color:expr, $font:expr) => {
        $job.append(
            $text,
            0.0,
            egui::text::TextFormat {
                font_id: $font.clone(),
                color: $color,
                ..Default::default()
            },
        );
    };
}

impl Widget for CycleDisplay<'_> {
    fn ui(self, ui: &mut Ui) -> Response {
        let font = FontId::monospace(14.0);
        let address_digits = self.arch.bus_chr_width();
        let mut inner_response = ui.interact(egui::Rect::NAN, ui.id().with("null"), egui::Sense::empty());

        // Draw ALE label (or blank space)
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;

            let ale = self.state.ale();
            if ale {
                *self.address_latch = self.state.address_bus;
            }
            let data_bus_value = self.data_bus_str();

            ui.horizontal(|ui| {
                ui.set_min_width(150.0);

                let mut job = LayoutJob::default();
                if ale {
                    job.append(
                        "A",
                        0.0,
                        TextFormat {
                            font_id: font.clone(),
                            color: ALE_COLOR,
                            ..Default::default()
                        },
                    );
                    job.append(
                        ":",
                        0.0,
                        TextFormat {
                            font_id: font.clone(),
                            color: COLON_COLOR,
                            ..Default::default()
                        },
                    );
                }
                else {
                    job.append(
                        "  ",
                        0.0,
                        TextFormat {
                            font_id: font.clone(),
                            color: Color32::TRANSPARENT,
                            ..Default::default()
                        },
                    )
                }
                ui.label(job);

                let mut job = LayoutJob::default();

                // Draw address bus label
                job.append(
                    &format!("{:0address_digits$X}", self.state.address_bus),
                    0.0,
                    TextFormat {
                        font_id: font.clone(),
                        color: TEXT_COLOR,
                        ..Default::default()
                    },
                );
                job.append(
                    ":",
                    0.0,
                    TextFormat {
                        font_id: font.clone(),
                        color: COLON_COLOR,
                        ..Default::default()
                    },
                );
                ui.label(job);

                // Draw data bus label
                if let Some(dbus_str) = self.data_bus_opt {
                    // Set text color if not valid hex
                    let valid_dbus = u16::from_str_radix(dbus_str.trim(), 16).is_ok();
                    let text_color = if valid_dbus { TEXT_COLOR } else { Color32::RED };

                    inner_response = ui.add_sized(
                        [40.0, ui.available_height()],
                        egui::TextEdit::singleline(dbus_str)
                            .font(TextStyle::Monospace)
                            .char_limit(4)
                            .text_color(text_color),
                    );
                }
                else {
                    let mut job = LayoutJob::default();
                    job.append(
                        &format!("{:04X}", self.state.data_bus),
                        0.0,
                        TextFormat {
                            font_id: font.clone(),
                            color: TEXT_COLOR,
                            ..Default::default()
                        },
                    );
                    ui.label(job);
                }
            });

            // Draw segment status or blank
            let mut job = LayoutJob::default();
            if self.arch.has_segment_status() {
                // TODO: Print segment status here
            }
            else {
                // Print four blank spaces
                job.append(
                    "    ",
                    0.0,
                    TextFormat {
                        font_id: font.clone(),
                        color: Color32::TRANSPARENT,
                        ..Default::default()
                    },
                );
                ui.label(job);
            }

            // Print memory activity

            let mut have_read = false;
            let mut have_write = false;
            let mut job = LayoutJob::default();
            let (mrdc_chr, mrdc_color) = if self.state.bus_command_bits & 0b0001 == 0 {
                have_read = true;
                ("R", READ_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };
            let (amwc_chr, amwc_color) = if self.state.bus_command_bits & 0b0010 == 0 {
                ("W", WRITE_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };
            let (mwtc_chr, mwtc_color) = if self.state.bus_command_bits & 0b0100 == 0 {
                have_write = true;
                ("W", WRITE_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };

            append_colored!(job, "M", MEM_COLOR, font);
            append_colored!(job, ":", COLON_COLOR, font);
            append_colored!(job, mrdc_chr, mrdc_color, font);
            append_colored!(job, amwc_chr, amwc_color, font);
            append_colored!(job, mwtc_chr, mwtc_color, font);
            append_colored!(job, " ", Color32::TRANSPARENT, font);
            ui.label(job);

            // Print IO activity
            let mut job = LayoutJob::default();
            let (iorc_chr, iorc_color) = if self.state.bus_command_bits & 0b1000 == 0 {
                have_read = true;
                ("R", READ_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };
            let (aiowc_chr, aiowc_color) = if self.state.bus_command_bits & 0b0001_0000 == 0 {
                ("W", WRITE_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };
            let (iowc_chr, iowc_color) = if self.state.bus_command_bits & 0b0010_0000 == 0 {
                have_write = true;
                ("W", WRITE_COLOR)
            }
            else {
                (".", DOT_COLOR)
            };

            append_colored!(job, "I", IO_COLOR, font);
            append_colored!(job, ":", COLON_COLOR, font);
            append_colored!(job, iorc_chr, iorc_color, font);
            append_colored!(job, aiowc_chr, aiowc_color, font);
            append_colored!(job, iowc_chr, iowc_color, font);
            append_colored!(job, " ", Color32::TRANSPARENT, font);
            ui.label(job);

            // Print pin states
            let mut job = LayoutJob::default();
            let bhe_chr = match self.state.bhe() {
                true => 'B',
                false => '.',
            };
            append_colored!(job, "P", PIN_COLOR, font);
            append_colored!(job, ":", COLON_COLOR, font);
            append_colored!(job, ".", TEXT_COLOR, font);
            append_colored!(job, ".", TEXT_COLOR, font);
            append_colored!(job, &bhe_chr.to_string(), TEXT_COLOR, font);
            ui.label(job);

            // Print bus status
            let mut job = LayoutJob::default();

            let status = self.arch.decode_status(self.state.cpu_status_bits);
            let color = BUS_STATE_COLORS[(status as usize) & 0x07];

            append_colored!(job, &format!("{}", status), color, font);
            append_colored!(job, "[", BRACKET_COLOR, font);
            append_colored!(job, &format!("{:1X}", (status as u8) & 0x07), TEXT_COLOR, font);
            append_colored!(job, "]", BRACKET_COLOR, font);
            append_colored!(job, " ", Color32::TRANSPARENT, font);
            ui.label(job);

            // Print T-state
            let mut job = LayoutJob::default();
            let t_cycle = (self.state.cpu_state_bits & 0x07) as usize;

            append_colored!(job, &format!("{}", T_STATES[t_cycle].0), T_STATES[t_cycle].1, font);
            append_colored!(job, " ", Color32::TRANSPARENT, font);
            ui.label(job);

            let bus_active = match self.arch {
                ServerCpuType::Intel80386 => {
                    // The 386 can write on t1
                    if self.state.is_writing() {
                        true
                    }
                    else {
                        // The 386 can read after T1
                        self.state.t_state() != TState::T1
                    }
                }
                ServerCpuType::Intel80286 => {
                    // The 286 can read/write after T1
                    self.state.t_state() != TState::T1
                }
                _ => {
                    // Older CPUs can only read/write in PASV state
                    status == BusState::PASV
                }
            };

            // Print read/write activity
            let mut job = LayoutJob::default();
            let mut printed_rw = false;
            if bus_active {
                if have_read {
                    append_colored!(job, "R", READ_COLOR, font);
                    append_colored!(job, "-> ", TEXT_COLOR, font);
                    append_colored!(job, &data_bus_value, TEXT_COLOR, font);
                    printed_rw = true;
                }
                else if have_write {
                    append_colored!(job, "<-", TEXT_COLOR, font);
                    append_colored!(job, "W ", WRITE_COLOR, font);
                    append_colored!(job, &data_bus_value, TEXT_COLOR, font);
                    printed_rw = true;
                }
            }

            if !printed_rw {
                append_colored!(job, "        ", Color32::TRANSPARENT, font);
            }
            ui.label(job);
        });

        inner_response
    }
}
