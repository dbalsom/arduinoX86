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
    events::{GuiEvent, GuiEventQueue},
    widgets::cycle_display::CycleDisplay,
    DEFAULT_FONT_SIZE,
};
use arduinox86_client::{ServerCpuType, ServerCycleLogPrinter, ServerCycleState};
use egui::{Color32, Margin, Response};

#[derive(Default)]
pub struct CycleTable {
    arch: ServerCpuType,
    cycles: Vec<ServerCycleState>,
    data_bus_str: String,
    address_latches: Vec<u32>,
}

impl CycleTable {
    pub fn new(arch: ServerCpuType) -> Self {
        CycleTable {
            arch,
            cycles: Vec::new(),
            data_bus_str: String::new(),
            address_latches: Vec::new(),
        }
    }

    pub fn set_arch(&mut self, arch: ServerCpuType) {
        self.arch = arch;
    }

    pub fn data_bus_str(&self) -> &str {
        &self.data_bus_str
    }

    pub fn set_cycles(&mut self, cycles: Vec<ServerCycleState>) {
        // Rows can be rendered out of order or skipped by the scroll area.
        // Resolve ALE once when the log changes, rather than during repaint.
        let mut latch = 0;
        self.address_latches.clear();
        self.address_latches.extend(cycles.iter().map(|cycle| {
            if cycle.ale() {
                latch = cycle.address_bus;
            }
            latch
        }));
        if cycles.is_empty() {
            self.data_bus_str.clear();
        }
        else {
            let last_cycle = cycles.last().unwrap();
            self.data_bus_str = format!("{:04X}", last_cycle.data_bus);
        }
        self.cycles = cycles;
    }

    pub fn push_cycle(&mut self, cycle: ServerCycleState) {
        let latch = if cycle.ale() {
            cycle.address_bus
        } else {
            self.address_latches.last().copied().unwrap_or(0)
        };
        self.address_latches.push(latch);
        self.data_bus_str = format!("{:04X}", cycle.data_bus);
        self.cycles.push(cycle);
    }

    pub fn cycles(&self) -> &[ServerCycleState] {
        &self.cycles
    }

    pub fn clear(&mut self) {
        self.cycles.clear();
        self.address_latches.clear();
        self.data_bus_str.clear();
    }

    fn show_rows(
        &mut self,
        ui: &mut egui::Ui,
        scroll: egui::ScrollArea,
    ) -> egui::scroll_area::ScrollAreaOutput<Option<Response>> {
        let row_height = ui.fonts(|fonts| fonts.row_height(&egui::FontId::monospace(DEFAULT_FONT_SIZE)))
            .max(ui.spacing().interact_size.y);
        scroll.show_rows(ui, row_height, self.cycles.len(), |ui, rows| {
            ui.set_min_width(ui.available_width());
            let mut response = None;
            for i in rows {
                let cycle = &self.cycles[i];
                let data = if i + 1 == self.cycles.len() && cycle.is_reading() {
                    Some(&mut self.data_bus_str)
                } else {
                    None
                };
                let mut latch = self.address_latches[i];
                // Keep the last row's editor ID stable while rows above it scroll.
                response = Some(ui.push_id(i, |ui| {
                    ui.add(CycleDisplay::new(self.arch, cycle.clone(), &mut latch, data))
                }).inner);
            }
            response
        })
    }

    pub fn show(&mut self, ui: &mut egui::Ui, events: &mut GuiEventQueue) -> Option<Response> {
        if self.cycles.is_empty() {
            ui.label("No cycles available");
            return None;
        }

        let bg: Color32 = ui.visuals().code_bg_color;
        let max_scroll_height = 400.0;

        let mut inner_response = None;

        ui.with_layout(egui::Layout::left_to_right(egui::Align::TOP), |ui| {
            ui.horizontal(|ui| {
                ui.label("Cycle Output:");
            });

            ui.with_layout(egui::Layout::right_to_left(egui::Align::TOP), |ui| {
                if ui
                    .button(egui::RichText::new(format!("{}", egui_phosphor::regular::X)).size(18.0))
                    .on_hover_text("Clear")
                    .clicked()
                {
                    events.push(GuiEvent::ClearCycleLog);
                    self.clear();
                }
                if ui
                    .button(egui::RichText::new(format!("{}", egui_phosphor::regular::CLIPBOARD_TEXT)).size(18.0))
                    .on_hover_text("Copy")
                    .clicked()
                {
                    let printer = ServerCycleLogPrinter::new(self.arch, &self.cycles);
                    ui.ctx().copy_text(printer.to_string());
                }
            });
        });

        egui::Frame::new()
            .fill(bg)
            .inner_margin(Margin::same(6))
            .outer_margin(Margin::ZERO)
            .show(ui, |ui| {
                let scroll = egui::ScrollArea::vertical()
                    .max_height(max_scroll_height)
                    .stick_to_bottom(true);
                inner_response = self.show_rows(ui, scroll).inner;
            });

        if let Some(resp) = inner_response.as_ref() {
            if resp.changed() {
                log::debug!("ClientWindow::show(): Response: {:?}", resp);
            }
        }

        inner_response
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cycle(address: u32, ale: bool) -> ServerCycleState {
        ServerCycleState {
            program_state: arduinox86_client::ProgramState::Execute,
            cpu_state_bits: 2,
            cpu_status_bits: 0x03,
            bus_control_bits: u8::from(ale),
            bus_command_bits: 0x7E,
            address_bus: address,
            data_bus: 0xABCD,
            pins: 0,
        }
    }

    fn draw(
        table: &mut CycleTable,
        ctx: &egui::Context,
        offset: f32,
        events: Vec<egui::Event>,
    ) -> (egui::scroll_area::ScrollAreaOutput<Option<Response>>, egui::FullOutput) {
        let mut listing = None;
        let output = ctx.run(egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1200.0, 700.0))),
            events,
            ..Default::default()
        }, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                listing = Some(table.show_rows(ui, egui::ScrollArea::vertical()
                    .max_height(400.0).animated(false).vertical_scroll_offset(offset)));
            });
        });
        (listing.unwrap(), output)
    }

    #[test]
    fn preserves_and_clears_ale_latch() {
        let mut table = CycleTable::new(ServerCpuType::Intel80286);
        table.set_cycles(vec![cycle(0x10001, true), cycle(0xABCDE0, false)]);
        table.push_cycle(cycle(0xFFFFF0, false));
        let mut latch = table.address_latches[2];
        assert_eq!(CycleDisplay::new(table.arch, table.cycles[2].clone(), &mut latch, None).data_bus_str(), "AB  ");
        table.push_cycle(cycle(0x20000, true));
        table.push_cycle(cycle(0x20001, false));
        let mut latch = table.address_latches[4];
        assert_eq!(CycleDisplay::new(table.arch, table.cycles[4].clone(), &mut latch, None).data_bus_str(), "ABCD");
        table.clear();
        assert!(table.data_bus_str().is_empty());
        table.push_cycle(cycle(0x30001, false));
        assert_eq!(table.address_latches, [0]);
        table.set_cycles(vec![cycle(0x40001, false)]);
        assert_eq!(table.address_latches, [0]);
    }

    #[test]
    fn virtualizes_large_cycle_logs() {
        let mut table = CycleTable::new(ServerCpuType::Intel80286);
        table.set_cycles((0..16384).map(|i| cycle(0x10000 + i, i % 4 == 0)).collect());
        let ctx = egui::Context::default();
        let (listing, _) = draw(&mut table, &ctx, 90000.0, Vec::new());
        assert!(listing.content_size.y > 16384.0 * 18.0);
        assert!(listing.state.offset.y > 80000.0);
        // Every address is unique: laying out the whole log would cache 16384
        // address galleys even though the painter clips them off-screen.
        assert!(ctx.fonts(|fonts| fonts.num_galleys_in_cache()) < 300);
    }

    #[test]
    fn editor_keeps_focus_after_scroll() {
        let mut table = CycleTable::new(ServerCpuType::Intel80286);
        table.set_cycles((0..100).map(|i| cycle(0x10000 + i, true)).collect());
        let ctx = egui::Context::default();
        draw(&mut table, &ctx, 1900.0, Vec::new());
        let (listing, _) = draw(&mut table, &ctx, 1900.0, Vec::new());
        let editor = listing.inner.unwrap();
        assert!(listing.inner_rect.intersects(editor.rect));
        ctx.memory_mut(|memory| memory.request_focus(editor.id));
        let (scrolled, _) = draw(&mut table, &ctx, 1800.0, vec![
            egui::Event::Key {
                key: egui::Key::A, physical_key: None, pressed: true, repeat: false,
                modifiers: egui::Modifiers { ctrl: true, command: true, ..Default::default() },
            },
            egui::Event::Text("1234".into()),
        ]);
        let updated = scrolled.inner.unwrap();
        assert_eq!(updated.id, editor.id);
        assert!(updated.changed());
        assert_eq!(table.data_bus_str(), "1234");
    }
}
