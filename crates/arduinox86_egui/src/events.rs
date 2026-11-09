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
use crate::enums::{ExecutionMode, FileOpenContext, FileSaveContext, MountAddress};
use arduinox86_client::CpuPin;
use std::path::PathBuf;

#[derive(PartialEq, Eq)]
pub enum GuiEvent {
    ResetState,
    LoadRegisters,
    EraseMemory,
    ReadMemory {
        address: u32,
        size:    u32,
    },
    RunProgram,
    Step,
    StepInstruction,
    ResetCpu,
    ContinueTrap,
    SetTrapEndpoint { cs: u16, ip: u16, enabled: bool },
    SetExecutionMode(ExecutionMode),
    Disconnect,
    SetFlag {
        flag:    u32,
        enabled: bool,
    },
    WritePin {
        pin:   CpuPin,
        value: bool,
    },
    WriteDataBus(u16),
    AssembleProgram {
        program_name: String,
    },
    PollStatus,
    ClearCycleLog,
    ToggleRefreshMemory {
        enabled: bool,
        hertz:   u32,
    },
    RefreshMemory,
    UploadBlob {
        blob_name: String,
        mount_address: MountAddress,
        size: Option<usize>,
    },
}

#[derive(Default)]
pub struct GuiEventQueue {
    pub events: Vec<GuiEvent>,
}

impl GuiEventQueue {
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    pub fn push(&mut self, event: GuiEvent) {
        self.events.push(event);
    }

    pub fn pop(&mut self) -> Option<GuiEvent> {
        if self.events.is_empty() {
            None
        }
        else {
            Some(self.events.remove(0))
        }
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }
}

pub enum FrontendThreadEvent {
    FileOpenDialogComplete {
        context: FileOpenContext,
        path: Option<PathBuf>,
        contents: Vec<u8>,
    },
    FileSaveDialogComplete(FileSaveContext),
    FileOpenError(FileOpenContext, String),
    FileSaveError(String),
    FileDialogCancelled,
    QuitRequested,
}
