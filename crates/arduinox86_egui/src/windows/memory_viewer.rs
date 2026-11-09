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
    controls::data_table::DataTableWidget,
    events::{GuiEvent, GuiEventQueue},
    TEXT_COLOR,
};
use egui::{Color32, TextStyle};

pub struct MemoryViewer {
    pub address_string: String,
    pub address: u32,
    pub size_string: String,
    pub size: u32,
    pub icon_size: f32,
    pub auto_refresh: bool,
    pub refresh_rate_string: String,
    pub refresh_rate: u32,
    pub dt: DataTableWidget,
}

impl Default for MemoryViewer {
    fn default() -> Self {
        Self {
            address_string: "000A0000".to_string(),
            address: 0xA0000,
            size_string: "00010000".to_string(), // Default to 64KB
            size: 0x10000,                       // Default to 64KB
            icon_size: 24.0,
            auto_refresh: false,
            refresh_rate_string: "1".to_string(),
            refresh_rate: 1,
            dt: DataTableWidget::default(),
        }
    }
}

impl MemoryViewer {
    pub fn new() -> Self {
        Self { ..Default::default() }
    }

    pub fn set_data(&mut self, data: &[u8]) {
        self.dt.set_data(data);
    }

    pub fn make_refresh_event(&self) -> GuiEvent {
        GuiEvent::ReadMemory {
            address: self.address,
            size:    self.size,
        }
    }

    pub fn show(&mut self, e_ctx: &egui::Context, c_ctx: &mut ClientContext, events: &mut GuiEventQueue) {
        egui::Window::new("Memory Viewer")
            .default_width(800.0)
            .default_height(600.0)
            .show(e_ctx, |ui| {
                ui.vertical(|ui| {
                    ui.horizontal(|ui| {
                        if ui
                            .add_enabled(
                                !c_ctx.busy,
                                egui::Button::new(
                                    egui::RichText::new(format!("{}", egui_phosphor::regular::DOWNLOAD_SIMPLE))
                                        .size(self.icon_size),
                                ),
                            )
                            .on_hover_text("Download Memory")
                            .clicked()
                        {
                            events.push(GuiEvent::ReadMemory {
                                address: self.address,
                                size:    self.size,
                            });
                        }

                        if ui
                            .add_enabled(
                                !c_ctx.busy,
                                egui::Button::new(
                                    egui::RichText::new(egui_phosphor::fill::ERASER).size(self.icon_size),
                                ),
                            )
                            .on_hover_text("Erase Memory")
                            .clicked()
                        {
                            events.push(GuiEvent::EraseMemory);
                        }

                        ui.separator();
                        ui.label("Address:");
                        if ui.text_edit_singleline(&mut self.address_string).lost_focus() {
                            if let Ok(addr) = u32::from_str_radix(&self.address_string, 16) {
                                self.address = addr;
                            }
                            else {
                                self.address = 0;
                            }
                        }
                        ui.label("Size:");
                        if ui.text_edit_singleline(&mut self.size_string).lost_focus() {
                            if let Ok(size) = u32::from_str_radix(&self.size_string, 16) {
                                self.size = size;
                            }
                            else {
                                self.size = 0;
                            }
                        }
                    });

                    ui.separator();

                    let mut new_check = self.auto_refresh;
                    ui.horizontal(|ui| {
                        ui.add(egui::Checkbox::new(&mut new_check, "Auto Refresh"));

                        if self.auto_refresh != new_check {
                            events.push(GuiEvent::ToggleRefreshMemory {
                                enabled: new_check,
                                hertz:   self.refresh_rate,
                            });
                            self.auto_refresh = new_check;
                        }

                        let text_color = if u32::from_str_radix(&self.refresh_rate_string, 10).is_ok() {
                            TEXT_COLOR
                        }
                        else {
                            Color32::RED
                        };

                        let response = ui.add(
                            egui::TextEdit::singleline(&mut self.refresh_rate_string)
                                .font(TextStyle::Monospace)
                                .desired_width(30.0)
                                .char_limit(2)
                                .text_color(text_color),
                        );

                        ui.label("Hz");

                        if response.lost_focus() {
                            if let Ok(rate) = self.refresh_rate_string.parse::<u32>() {
                                self.refresh_rate = rate;
                                if self.auto_refresh {
                                    events.push(GuiEvent::ToggleRefreshMemory {
                                        enabled: false,
                                        hertz:   1,
                                    });
                                    events.push(GuiEvent::ToggleRefreshMemory {
                                        enabled: true,
                                        hertz:   rate,
                                    });
                                }
                            }
                            else {
                                self.refresh_rate = 1;
                            }
                        }
                    });

                    self.dt.show(ui);
                });
            });
    }
}
