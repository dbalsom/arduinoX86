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
    enums::MountAddress,
    events::{GuiEvent, GuiEventQueue},
    structs::BinaryBlob,
    widgets::mount_address_widget::MountAddressWidget,
    TEXT_COLOR,
};
use egui::{Color32, TextStyle};

pub struct BinaryView {
    pub name: String,
    pub icon_size: f32,
    pub mount_addr: MountAddress,
    pub mount_str: String,
    pub size_str: String,
    pub size: usize,
    pub dt: DataTableWidget,
}

impl Default for BinaryView {
    fn default() -> Self {
        Self {
            name: "Program".into(),
            icon_size: 24.0,
            mount_addr: MountAddress::CsIp,
            mount_str: "0".to_string(),
            size_str: "0".to_string(),
            size: 0,
            dt: DataTableWidget::default(),
        }
    }
}

impl BinaryView {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.into(),
            ..Default::default()
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn set_data(&mut self, data: &[u8]) {
        self.size = data.len();
        self.size_str = format!("{}", self.size);
        self.dt.set_data(data);
    }

    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        blob: &mut BinaryBlob,
        c_ctx: &mut ClientContext,
        events: &mut GuiEventQueue,
    ) {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(
                        !c_ctx.busy,
                        egui::Button::new(
                            egui::RichText::new(format!("{}", egui_phosphor::regular::BOX_ARROW_UP))
                                .size(self.icon_size),
                        ),
                    )
                    .on_hover_text("Load binary")
                    .clicked()
                {
                    events.push(GuiEvent::UploadBlob {
                        blob_name: self.name.clone(),
                        mount_address: self.mount_addr,
                        size: if self.size == 0 { None } else { Some(self.size) },
                    });
                }

                ui.separator();
                ui.add(MountAddressWidget::new(&mut self.mount_addr, &mut self.mount_str));

                let text_color = if self.size_str.parse::<usize>().is_ok() {
                    TEXT_COLOR
                }
                else {
                    Color32::RED
                };

                ui.label("Size:");
                ui.add(
                    egui::TextEdit::singleline(&mut self.size_str)
                        .font(TextStyle::Monospace)
                        .desired_width(50.0)
                        .char_limit(6)
                        .text_color(text_color),
                );

                if let Ok(size) = self.size_str.parse::<usize>() {
                    self.size = size;
                }
            });

            blob.set_mount_address(self.mount_addr.clone());
            ui.separator();
            self.dt.show(ui);
        });
    }
}
