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

use crate::disassembly::{
    validate_request, DisassemblyReadout, DisassemblyRequest, DisassemblyRow, MAX_DISASSEMBLY_BYTES,
};
use arduinox86_client::{
    trap::{InstructionState, TrapStatus},
    Registers32, RemoteCpuRegisters, ServerCpuType,
};

pub struct DisassemblyWindow {
    cpu: ServerCpuType,
    address_text: String,
    size_text: String,
    base_text: String,
    base: u32,
    mapped_cs: Option<u16>,
    mapping_source: &'static str,
    follow: bool,
    bits32: bool,
    current: Option<InstructionState>,
    rows: Vec<DisassemblyRow>,
    max_bytes_chars: usize,
    max_instruction_chars: usize,
    displayed: Option<DisassemblyRequest>,
    generation: u64,
    pending: Option<DisassemblyRequest>,
    submitted_id: Option<u64>,
    loading: bool,
    message: Option<String>,
    follow_scroll_pending: bool,
    reset_scroll: bool,
    context_rows: usize,
}

impl Default for DisassemblyWindow {
    fn default() -> Self {
        Self {
            cpu: ServerCpuType::Undetected,
            address_text: "000000".into(),
            size_text: "0100".into(),
            base_text: "000000".into(),
            base: 0,
            mapped_cs: None,
            mapping_source: "unset",
            follow: true,
            bits32: false,
            current: None,
            rows: Vec::new(),
            max_bytes_chars: 17, // Six bytes, including spaces.
            max_instruction_chars: 0,
            displayed: None,
            generation: 0,
            pending: None,
            submitted_id: None,
            loading: false,
            message: None,
            follow_scroll_pending: false,
            reset_scroll: true,
            context_rows: 8,
        }
    }
}

impl DisassemblyWindow {
    pub fn connect(&mut self, cpu: ServerCpuType) {
        self.clear();
        self.cpu = cpu;
        self.bits32 = false;
    }

    fn cancel_read(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        self.pending = None;
        self.loading = false;
        self.message = None;
    }

    fn invalidate(&mut self) {
        self.cancel_read();
        self.rows.clear();
        self.max_bytes_chars = 17;
        self.max_instruction_chars = 0;
        self.displayed = None;
        self.follow_scroll_pending = false;
        self.reset_scroll = true;
    }

    pub fn clear(&mut self) {
        self.invalidate();
        self.current = None;
        self.mapped_cs = None;
        self.mapping_source = "unset";
    }

    pub fn execution_started(&mut self) {
        if let Some(current) = &mut self.current {
            current.status = TrapStatus::Running;
        }
        if self.follow {
            // Keep the listing and scroll position while the next instruction runs.
            self.cancel_read();
            self.follow_scroll_pending = false;
        }
    }

    /// Seed only from a successful load, never from live edits or handler registers.
    pub fn loaded(&mut self, regs: &RemoteCpuRegisters) {
        self.clear();
        let (cs, base) = match regs {
            RemoteCpuRegisters::V1(r) => (r.cs, u32::from(r.cs) << 4),
            RemoteCpuRegisters::V2(r) => (r.cs, r.cs_desc.base_address()),
            RemoteCpuRegisters::V3(r) => (r.cs(), r.cs_base()),
        };
        self.mapped_cs = Some(cs);
        self.base = base;
        self.base_text = format!("{:0width$X}", base, width = self.cpu.bus_chr_width());
        self.mapping_source = "loaded";
        self.address_text = format!("{:0width$X}", regs.code_address(), width = self.cpu.bus_chr_width());
        if !self.follow {
            self.refresh();
        }
    }

    pub fn observe(&mut self, state: &InstructionState) {
        if state.status == TrapStatus::Idle {
            if self.current.is_some() {
                self.clear();
            }
            return;
        }
        if self.current.as_ref() == Some(state) {
            return;
        }
        if self.current.as_ref().is_some_and(|old| old.run_id != state.run_id) {
            // A load performed outside this GUI has no trustworthy base mapping.
            self.clear();
        }
        let changed = self.current.as_ref().map_or(true, |old| {
            (old.run_id, old.sequence, old.resume_cs, old.resume_ip, old.status)
                != (
                    state.run_id,
                    state.sequence,
                    state.resume_cs,
                    state.resume_ip,
                    state.status,
                )
        });
        self.current = Some(state.clone());
        if self.follow && changed {
            self.refresh();
        }
    }

    fn observed_address(&self) -> Option<(u16, u16)> {
        self.current
            .as_ref()
            .filter(|s| {
                matches!(s.status, TrapStatus::Ready | TrapStatus::Paused | TrapStatus::Captured)
                    || (s.status == TrapStatus::Running && s.has_frame())
            })
            .map(|s| (s.resume_cs, s.resume_ip))
    }

    fn current_physical(&self) -> Option<u32> {
        let (cs, ip) = self.observed_address()?;
        if self.mapped_cs != Some(cs) {
            return None;
        }
        self.base.checked_add(u32::from(ip))
    }

    fn parse_hex(text: &str) -> Option<u32> {
        let text = text.trim();
        u32::from_str_radix(
            text.strip_prefix("0x")
                .or_else(|| text.strip_prefix("0X"))
                .unwrap_or(text),
            16,
        )
        .ok()
    }

    pub fn refresh(&mut self) {
        self.cancel_read();
        match self.make_request() {
            Ok(request) => {
                self.address_text = format!("{:0width$X}", request.address, width = self.cpu.bus_chr_width());
                self.pending = Some(request);
                self.loading = true;
            }
            Err(message) => {
                self.invalidate();
                self.message = Some(message);
            }
        }
    }

    // Retain known instruction boundaries. Near the end of the read buffer,
    // carry enough preceding rows into the next read to keep the arrow halfway down.
    fn follow_start(&self, address: u32, ip: u32, cs: u16, size: u32) -> (u32, u32) {
        let compatible = self.displayed.is_some_and(|request| {
            request.cs == Some(cs)
                && request.bits32 == self.bits32
                && request.address.checked_sub(request.ip) == Some(self.base)
        });
        if compatible {
            let first = match self.rows.iter().position(|row| row.address == address) {
                Some(index) if index + self.context_rows + 1 >= self.rows.len() => {
                    Some(index.saturating_sub(self.context_rows))
                }
                Some(_) => Some(0),
                None if self.rows.last().is_some_and(|row| row.address < address) => {
                    Some(self.rows.len().saturating_sub(self.context_rows))
                }
                _ => None,
            };
            if let Some(first) = first {
                if let Some(row) = self.rows[first..]
                    .iter()
                    .find(|row| row.address <= address && address - row.address < size)
                {
                    return (row.address, row.ip);
                }
            }
        }
        (address, ip)
    }

    fn make_request(&self) -> Result<DisassemblyRequest, String> {
        let Some(mut size) = Self::parse_hex(&self.size_text).filter(|size| (1..=MAX_DISASSEMBLY_BYTES).contains(size))
        else {
            return Err("Size must be 1..1000 hex bytes.".into());
        };
        let (address, ip, cs) = if self.follow {
            let Some((cs, ip)) = self.observed_address() else {
                return Err("Waiting for a trap instruction address. Uncheck Follow to browse memory.".into());
            };
            let Some(address) = self.current_physical() else {
                return Err(format!("Set a CS base for {cs:04X} to follow this address."));
            };
            let (address, ip) = self.follow_start(address, u32::from(ip), cs, size);
            // Do not read past the 16-bit code-offset boundary into unrelated memory.
            size = size.min(0x10000 - ip);
            (address, ip, Some(cs))
        } else {
            let Some(address) = Self::parse_hex(&self.address_text) else {
                return Err("Enter a hexadecimal memory address.".into());
            };
            let offset = address.checked_sub(self.base);
            let mapped = self.mapped_cs.is_some() && offset.is_some_and(|ip| self.bits32 || ip <= 0xffff);
            if mapped {
                let ip = offset.unwrap();
                if !self.bits32 {
                    size = size.min(0x10000 - ip);
                }
                (address, ip, self.mapped_cs)
            } else {
                (address, address, None)
            }
        };
        // A reset vector is only 16 bytes below the end of the physical bus.
        // Clip manual reads too; use wide arithmetic for the end-exclusive limit.
        let remaining = (1u64 << self.cpu.address_bus_size()).saturating_sub(u64::from(address));
        size = u64::from(size).min(remaining) as u32;
        let request = DisassemblyRequest {
            id: self.generation,
            address,
            size,
            ip,
            cs,
            bits32: self.bits32,
        };
        validate_request(self.cpu, request)?;
        Ok(request)
    }

    pub fn pending_request(&self) -> Option<DisassemblyRequest> {
        self.pending
    }

    pub fn submitted(&mut self, request: DisassemblyRequest) {
        self.submitted_id = Some(request.id);
        if self.pending == Some(request) {
            self.pending = None;
        }
    }

    pub fn apply(&mut self, readout: DisassemblyReadout) {
        if self.submitted_id == Some(readout.request.id) {
            self.submitted_id = None;
        }
        if readout.request.id != self.generation {
            return;
        }
        self.loading = false;
        match readout.result {
            Ok(rows) => {
                // Changed instruction lengths can hide the observed IP inside a
                // decoded row. Restart at that known boundary instead of guessing.
                if self.follow
                    && self.current_physical().is_some_and(|address| {
                        address != readout.request.address && !rows.iter().any(|row| row.address == address)
                    })
                {
                    self.invalidate();
                    self.refresh();
                    return;
                }
                self.reset_scroll |= self.displayed.map(|old| (old.address, old.cs, old.bits32))
                    != Some((readout.request.address, readout.request.cs, readout.request.bits32));
                // Measure text lengths only when bytes change. Off-screen rows
                // still contribute to column widths without being laid out.
                (self.max_bytes_chars, self.max_instruction_chars) =
                    rows.iter().fold((17, 0), |(bytes, instruction), row| {
                        (
                            bytes.max(row.bytes.chars().count()),
                            instruction.max(row.instruction.chars().count()),
                        )
                    });
                self.rows = rows;
                self.displayed = Some(readout.request);
                self.follow_scroll_pending = self.follow;
            }
            Err(error) => self.message = Some(error),
        }
    }

    pub fn read_failed(&mut self, message: String) {
        if self.submitted_id.take() == Some(self.generation) {
            self.loading = false;
            self.message = Some(message);
        }
    }

    fn apply_base(&mut self) {
        let Some(base) =
            Self::parse_hex(&self.base_text).filter(|base| u64::from(*base) < (1u64 << self.cpu.address_bus_size()))
        else {
            self.message = Some("CS base is outside the CPU address bus.".into());
            return;
        };
        self.base = base;
        self.mapped_cs = self.current.as_ref().map(|s| s.resume_cs).or(self.mapped_cs);
        self.mapping_source = "manual";
        self.invalidate();
        self.refresh();
    }

    pub fn show(&mut self, ctx: &egui::Context) {
        egui::Window::new("Disassembly")
            .default_width(760.0)
            .default_height(440.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Address (hex):");
                    ui.add_enabled(!self.follow, egui::TextEdit::singleline(&mut self.address_text)
                        .font(egui::TextStyle::Monospace).desired_width(90.0).char_limit(10));
                    ui.label("Bytes (hex):");
                    ui.add(egui::TextEdit::singleline(&mut self.size_text)
                        .font(egui::TextStyle::Monospace).desired_width(60.0).char_limit(6));
                    if ui.button("Read").clicked() {
                        self.refresh();
                    }
                    if ui.checkbox(&mut self.follow, "Follow CS:IP").changed() {
                        self.refresh();
                    }
                });

                ui.horizontal(|ui| {
                    let cs = self.current.as_ref().map(|s| s.resume_cs).or(self.mapped_cs);
                    ui.label(cs.map_or("CS base:".into(), |cs| format!("CS {cs:04X} base:")));
                    ui.add(egui::TextEdit::singleline(&mut self.base_text)
                        .font(egui::TextStyle::Monospace).desired_width(90.0).char_limit(10));
                    if ui.button("Apply base").clicked() {
                        self.apply_base();
                    }
                    let source = self.mapped_cs.map_or(self.mapping_source.to_string(),
                        |cs| format!("{} for CS {cs:04X}", self.mapping_source));
                    ui.label(format!("({source})")).on_hover_text(
                        "Address mapping only. Update the base if IRET or guest code changes CS; the server does not report the guest CS cache.");
                    ui.separator();
                    ui.label(format!("{}", self.cpu));
                    if matches!(self.cpu, ServerCpuType::Intel80386) {
                        if ui.checkbox(&mut self.bits32, "32-bit code").changed() {
                            self.refresh();
                        }
                    }
                });

                ui.horizontal(|ui| {
                    if let Some(state) = &self.current {
                        ui.label(format!("Last observed return {:04X}:{:04X}  |  {:?}  |  Trap #{}",
                            state.resume_cs, state.resume_ip, state.status, state.sequence));
                    } else {
                        ui.label("No trap instruction address available.");
                    }
                    if self.loading {
                        ui.spinner().on_hover_text("Reading disassembly…");
                    }
                });

                if let Some(message) = &self.message {
                    ui.label(message);
                }

                ui.separator();
                self.show_listing(ui);
            });
    }

    fn show_listing(&mut self, ui: &mut egui::Ui) -> egui::scroll_area::ScrollAreaOutput<Option<egui::Rect>> {
        let current = self.current_physical();
        let width = self.cpu.bus_chr_width();
        let ip_width = if self.bits32 { 8 } else { 4 };
        let row_height = ui
            .text_style_height(&egui::TextStyle::Monospace)
            .max(ui.text_style_height(&egui::TextStyle::Body))
            .max(ui.spacing().interact_size.y);
        let pitch = row_height + 3.0;
        let char_width = ui.fonts(|fonts| fonts.glyph_width(&egui::TextStyle::Monospace.resolve(ui.style()), '0'));
        let cs = self.displayed.and_then(|r| r.cs);
        let titles = ["Address", "CS:IP", "Bytes", "Instruction"];
        
        let mut column_widths = [
            16.0 + width as f32 * char_width,
            (ip_width + if cs.is_some() { 5 } else { 0 }) as f32 * char_width,
            self.max_bytes_chars as f32 * char_width,
            self.max_instruction_chars as f32 * char_width,
        ];

        for (width, title) in column_widths.iter_mut().zip(titles) {
            *width = width.max(
                ui.painter()
                    .layout_no_wrap(
                        title.into(),
                        egui::TextStyle::Body.resolve(ui.style()),
                        ui.visuals().strong_text_color(),
                    )
                    .size()
                    .x,
            );
        }

        let content_width = column_widths.iter().sum::<f32>() + 3.0 * 16.0;
        let selected = current.and_then(|address| self.rows.binary_search_by_key(&address, |row| row.address).ok());
        let mut scroll = egui::ScrollArea::both().auto_shrink([false, false]).animated(false);

        if self.reset_scroll {
            scroll = scroll.vertical_scroll_offset(0.0);
            self.reset_scroll = false;
        }
        
        scroll.show_viewport(ui, |ui, viewport| {
            let origin = ui.max_rect().min;
            let visible = viewport.translate(origin.to_vec2());
            self.context_rows = (viewport.height() / (2.0 * pitch)).ceil() as usize;
            // Reserve the full scroll extent, but create widgets only for visible
            // rows. Off-screen instructions still contribute to the column widths.
            let padding = if self.follow { viewport.height() * 0.5 } else { 0.0 };

            ui.set_min_size(egui::vec2(
                content_width,
                (self.rows.len() + 1) as f32 * pitch - 3.0 + padding,
            ));

            let cells = |index: usize| {
                let mut x = origin.x;
                column_widths.map(|width| {
                    let rect = egui::Rect::from_min_size(
                        egui::pos2(x, origin.y + index as f32 * pitch),
                        egui::vec2(width, row_height),
                    );
                    x += width + 16.0;
                    rect
                })
            };

            if viewport.min.y < row_height {
                for (rect, title) in cells(0).into_iter().zip(titles) {
                    ui.put(
                        rect,
                        egui::Label::new(egui::RichText::new(title).strong())
                            .extend()
                            .halign(egui::Align::Min),
                    );
                }
            }

            let first = ((viewport.min.y / pitch).floor() as usize)
                .saturating_sub(1)
                .min(self.rows.len());
            let end = ((viewport.max.y / pitch).ceil() as usize).min(self.rows.len());

            for index in first..end {
                let row = &self.rows[index];
                let selected = selected == Some(index);
                let color = if selected {
                    ui.visuals().selection.stroke.color
                } else {
                    ui.visuals().text_color()
                };

                let [mut address, logical, bytes, instruction] = cells(index + 1);

                if index % 2 == 0 {
                    ui.painter().rect_filled(
                        egui::Rect::from_min_size(address.min, egui::vec2(content_width, row_height))
                            .expand2(egui::vec2(0.0, 1.5)),
                        0.0,
                        ui.visuals().faint_bg_color,
                    );
                }

                ui.push_id(("instruction", index), |ui| {
                    let marker = egui::Rect::from_min_size(address.min, egui::vec2(12.0, row_height));
                    ui.put(
                        marker,
                        egui::Label::new(egui::RichText::new(if selected { "▶" } else { "" }).color(color))
                            .extend()
                            .halign(egui::Align::Min),
                    );

                    address.min.x += 16.0;
                    let address_text = format!("{:0width$X}", row.address);
                    let logical_text = cs.map_or_else(
                        || format!("{:0ip_width$X}", row.ip),
                        |cs| format!("{cs:04X}:{:0ip_width$X}", row.ip),
                    );

                    for (rect, text) in [
                        (address, address_text.as_str()),
                        (logical, logical_text.as_str()),
                        (bytes, row.bytes.as_str()),
                        (instruction, row.instruction.as_str()),
                    ] {
                        ui.put(
                            rect,
                            egui::Label::new(egui::RichText::new(text).monospace().color(color))
                                .extend()
                                .halign(egui::Align::Min),
                        );
                    }
                });
            }
            
            // Locate the target even if it is off-screen and wasn't rendered.
            let selected_row = selected.map(|index| cells(index + 1)[0]);
            if self.follow {
                if self.follow_scroll_pending && !self.loading {
                    if let Some(row) = selected_row {
                        if row.center().y > visible.center().y || row.top() < visible.top() {
                            // Follow vertically without disturbing horizontal browsing.
                            ui.scroll_with_delta(egui::vec2(0.0, visible.center().y - row.center().y));
                        }
                    }
                    self.follow_scroll_pending = false;
                }
            }
            selected_row
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::disassembly::decode;
    use arduinox86_client::RemoteCpuRegistersV2;

    fn fixture() -> DisassemblyWindow {
        let mut view = DisassemblyWindow::default();
        view.connect(ServerCpuType::Intel80286);
        let mut regs = RemoteCpuRegistersV2 {
            cs: 0x18,
            ip: 0x100,
            ..Default::default()
        };
        regs.cs_desc.set_base_address(0x12003);
        view.loaded(&RemoteCpuRegisters::V2(regs));
        view
    }

    fn state(cs: u16, ip: u16, sequence: u32) -> InstructionState {
        let mut bytes = [0; InstructionState::WIRE_SIZE];
        bytes[..4].copy_from_slice(&[1, 3, 1, 1]);
        bytes[4..8].copy_from_slice(&1u32.to_le_bytes());
        bytes[8..12].copy_from_slice(&sequence.to_le_bytes());
        bytes[16..18].copy_from_slice(&cs.to_le_bytes());
        bytes[18..20].copy_from_slice(&ip.to_le_bytes());
        bytes[26] = 6;
        bytes[27] = InstructionState::FRAME_VALID | InstructionState::PROTECTED_MODE;
        InstructionState::from_bytes(&bytes).unwrap()
    }

    fn reply(request: DisassemblyRequest) -> DisassemblyReadout {
        DisassemblyReadout {
            request,
            result: decode(ServerCpuType::Intel80286, request, &vec![0x90; request.size as usize]),
        }
    }

    fn finish_read(view: &mut DisassemblyWindow) -> DisassemblyRequest {
        let request = view.pending_request().unwrap();
        view.submitted(request);
        view.apply(reply(request));
        request
    }

    fn draw_listing(
        view: &mut DisassemblyWindow,
        ctx: &egui::Context,
    ) -> (
        egui::scroll_area::ScrollAreaOutput<Option<egui::Rect>>,
        egui::FullOutput,
    ) {
        let mut listing = None;
        let output = ctx.run(
            egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(900.0, 360.0))),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    listing = Some(view.show_listing(ui));
                });
            },
        );
        (listing.unwrap(), output)
    }

    #[test]
    fn arrow_scrolls_at_viewport_midpoint() {
        let mut view = fixture();
        let ctx = egui::Context::default();
        view.observe(&state(0x18, 0x100, 1));
        let initial = finish_read(&mut view);
        // Settle egui's first-frame grid measurements.
        draw_listing(&mut view, &ctx);
        let (first, _) = draw_listing(&mut view, &ctx);
        assert_eq!(first.state.offset.y, 0.0);
        let first_y = first.inner.unwrap().center().y;
        view.execution_started();
        assert!(!view.rows.is_empty(), "stepping must retain the listing");
        view.observe(&state(0x18, 0x101, 2));
        assert_eq!(finish_read(&mut view).address, initial.address);
        let (second, _) = draw_listing(&mut view, &ctx);
        assert_eq!(second.state.offset.y, 0.0);
        assert!(second.inner.unwrap().center().y > first_y);
        for ip in 0x102..=0x120 {
            view.execution_started();
            view.observe(&state(0x18, ip, u32::from(ip)));
            assert_eq!(finish_read(&mut view).address, initial.address);
            draw_listing(&mut view, &ctx);
        }
        let (middle, _) = draw_listing(&mut view, &ctx);
        assert!(middle.state.offset.y > 0.0);
        assert!(
            (middle.inner.unwrap().center().y - middle.inner_rect.center().y).abs() < 2.0,
            "marker {:?}, viewport {:?}, offset {:?}",
            middle.inner,
            middle.inner_rect,
            middle.state.offset
        );
        view.observe(&state(0x18, 0x121, 0x121));
        finish_read(&mut view);
        draw_listing(&mut view, &ctx);
        let (next, _) = draw_listing(&mut view, &ctx);
        assert!(next.state.offset.y > middle.state.offset.y);
        assert!((next.inner.unwrap().center().y - next.inner_rect.center().y).abs() < 2.0);
        // A backward branch stays in the same listing and returns the marker to view.
        view.observe(&state(0x18, 0x102, 0x122));
        assert_eq!(finish_read(&mut view).address, initial.address);
        draw_listing(&mut view, &ctx);
        let (back, _) = draw_listing(&mut view, &ctx);
        assert_eq!(back.state.offset.y, 0.0);
        assert!(back.inner_rect.contains(back.inner.unwrap().center()));
        // Browsing manually must not follow later instruction observations.
        view.follow = false;
        view.observe(&state(0x18, 0x140, 0x123));
        assert!(view.pending_request().is_none());
        let (manual, _) = draw_listing(&mut view, &ctx);
        assert_eq!(manual.state.offset.y, back.state.offset.y);
    }

    #[test]
    fn rolling_reads_preserve_and_resync() {
        let mut view = fixture();
        view.size_text = "20".into();
        view.context_rows = 4;
        view.observe(&state(0x18, 0x100, 1));
        finish_read(&mut view);
        view.observe(&state(0x18, 0x11e, 2));
        let rolled = finish_read(&mut view);
        assert_eq!(rolled.ip, 0x11a);
        assert_eq!(view.rows[4].ip, 0x11e);
        view.observe(&state(0x18, 0x300, 3));
        assert_eq!(finish_read(&mut view).ip, 0x300);
        view.observe(&state(0x18, 0x301, 4));
        let request = view.pending_request().unwrap();
        assert_eq!(request.ip, 0x300);
        view.submitted(request);
        let mut changed = vec![0x90; request.size as usize];
        changed[..3].copy_from_slice(&[0xB8, 0x34, 0x12]);
        view.apply(DisassemblyReadout {
            request,
            result: decode(ServerCpuType::Intel80286, request, &changed),
        });
        assert_eq!(
            view.pending_request().unwrap().ip,
            0x301,
            "a known return address overrides stale decoding boundaries"
        );
    }

    #[test]
    fn virtual_rows_keep_widths_and_follow() {
        let mut view = fixture();
        view.size_text = "1000".into();
        view.observe(&state(0x18, 0x100, 1));
        let request = view.pending_request().unwrap();
        view.submitted(request);
        let mut readout = reply(request);
        let last = readout.result.as_mut().unwrap().last_mut().unwrap();
        last.bytes = "26 ".repeat(14) + "90";
        last.instruction = "rep es: ".to_string() + &"long instruction ".repeat(12);
        view.apply(readout);
        let ctx = egui::Context::default();
        draw_listing(&mut view, &ctx);
        let (first, _) = draw_listing(&mut view, &ctx);
        assert!(first.content_size.x > first.inner_rect.width());
        assert!(first.content_size.y > 4096.0 * 18.0);
        assert!(ctx.fonts(|fonts| fonts.num_galleys_in_cache()) < 150);
        let mut scroll_state = first.state;
        scroll_state.offset.x = 100.0;
        scroll_state.store(&ctx, first.id);
        // A distant target within the retained listing must be found without
        // rendering the thousands of rows between it and the old viewport.
        view.current = Some(state(0x18, 0x900, 2));
        view.follow_scroll_pending = true;
        draw_listing(&mut view, &ctx);
        let (jump, _) = draw_listing(&mut view, &ctx);
        assert!(jump.state.offset.y > 30000.0);
        assert!((jump.inner.unwrap().center().y - jump.inner_rect.center().y).abs() < 2.0);
        assert_eq!(jump.state.offset.x, 100.0);
        assert_eq!(jump.content_size.x, first.content_size.x);
        assert!(ctx.fonts(|fonts| fonts.num_galleys_in_cache()) < 200);
    }

    #[test]
    fn follows_return_rejecting_stale_reads() {
        let mut view = fixture();
        let mut s = state(0x18, 0x103, 1);
        view.observe(&s);
        let first = view.pending_request().unwrap();
        assert_eq!((first.address, first.ip, first.cs), (0x12106, 0x103, Some(0x18)));
        view.submitted(first);
        view.execution_started();
        s.resume_ip = 0x100; // A taken backward branch must move the view back.
        s.sequence = 2;
        view.observe(&s);
        let second = view.pending_request().unwrap();
        view.apply(reply(first));
        assert!(view.rows.is_empty());
        view.submitted(second);
        view.apply(reply(second));
        assert_eq!(view.rows[0].address, 0x12103);
        assert_eq!(view.current_physical(), Some(view.rows[0].address));
        view.observe(&s);
        assert!(view.pending_request().is_none()); // Identical polls do not reread.
        s.sequence += 1; // REP may trap again at the same saved IP.
        view.observe(&s);
        assert!(view.pending_request().is_some());
    }

    #[test]
    fn selector_change_requires_mapping() {
        let mut view = fixture();
        view.observe(&state(0x28, 0x200, 1));
        assert!(view.pending_request().is_none());
        assert!(view.message.as_ref().unwrap().contains("0028"));
        view.base_text = "0x18007".into();
        view.apply_base();
        let request = view.pending_request().unwrap();
        assert_eq!(request.address, 0x18207);
        assert_eq!(request.cs, Some(0x28));
        view.submitted(request);
        view.clear();
        view.apply(reply(request));
        assert!(view.rows.is_empty() && view.current.is_none() && view.mapped_cs.is_none());
        view.observe(&state(0x28, 0x200, 2));
        assert!(view.pending_request().is_none());
    }

    #[test]
    fn manual_browsing_respects_segment_end() {
        let mut view = fixture();
        view.observe(&state(0x18, 0xffff, 1));
        assert_eq!(view.pending_request().unwrap().size, 1);
        view.follow = false;
        view.address_text = "28000".into();
        view.size_text = "20".into();
        view.refresh();
        let manual = view.pending_request().unwrap();
        view.submitted(manual);
        view.apply(reply(manual));
        view.observe(&state(0x18, 0x104, 2));
        assert_eq!(view.rows[0].address, 0x28000);
        assert!(view.pending_request().is_none());
        view.size_text = "FFFFFFFF".into();
        view.refresh();
        assert!(view.pending_request().is_none() && view.message.is_some());
    }

    #[test]
    fn clips_manual_reads_at_bus_end() {
        for (cpu, reset_vector) in [
            (ServerCpuType::Intel8088, 0xFFFF0),
            (ServerCpuType::Intel80286, 0xFFFFF0),
            (ServerCpuType::Intel80386, 0x3FFFFF0), // 386EX: 26 address pins.
        ] {
            let mut view = DisassemblyWindow::default();
            view.connect(cpu);
            view.follow = false;
            view.address_text = format!("{reset_vector:X}");
            view.refresh();
            let request = view.pending_request().unwrap();
            assert_eq!((request.address, request.size), (reset_vector, 16));
            view.address_text = format!("{:X}", reset_vector + 15);
            view.refresh();
            assert_eq!(view.pending_request().unwrap().size, 1);
        }
        let mut view = fixture();
        view.follow = false;
        view.address_text = "1000000".into();
        view.refresh();
        assert!(view.pending_request().is_none() && view.message.is_some());
    }

    #[test]
    fn clips_286_reset_vector_to_16_bytes() {
        let mut view = fixture();
        let mut regs = RemoteCpuRegistersV2 {
            cs: 0xF000,
            ip: 0xFFF0,
            ..Default::default()
        };
        regs.cs_desc.set_base_address(0xFF0000);
        view.loaded(&RemoteCpuRegisters::V2(regs));
        let mut loaded = state(0xF000, 0xFFF0, 0);
        loaded.status = TrapStatus::Ready;
        view.observe(&loaded);
        let request = finish_read(&mut view);
        assert_eq!((request.address, request.ip, request.size), (0xFFFFF0, 0xFFF0, 16));
        assert_eq!(view.rows.last().unwrap().address, 0xFFFFFF);
    }

    #[test]
    fn follows_observed_running_boundary() {
        let mut view = fixture();
        let mut s = state(0x18, 0x104, 2);
        s.status = TrapStatus::Running;
        view.observe(&s);
        assert_eq!(view.pending_request().unwrap().address, 0x12107);
        s.status = TrapStatus::Failed;
        view.observe(&s);
        assert!(view.pending_request().is_none());
    }
}
