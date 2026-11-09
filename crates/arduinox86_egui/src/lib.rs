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
#![warn(clippy::all, rust_2018_idioms)]

mod app;
mod assembler;
mod async_exec;
mod character_encoding;
mod client;
mod config;
mod controls;
mod disassembly;
mod enums;
mod events;
mod file_dialogs;
mod memory_upload;
mod range_check;
mod register_state;
mod resource_manager;
mod scheduler;
mod serial_manager;
mod serial_worker;
mod structs;
mod style;
mod thread_event;
mod widgets;
mod window_manager;
mod windows;

pub use app::App;
use eframe::epaint::Color32;

pub const DEFAULT_FONT_SIZE: f32 = 14.0;
pub const TEXT_COLOR: Color32 = Color32::from_rgba_premultiplied(0xda, 0xda, 0xda, 0xff);
