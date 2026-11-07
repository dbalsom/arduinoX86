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
use crate::enums::ClientControlState;
use arduinox86_client::{
    ProgramState,
    RemoteCpuRegisters,
    RemoteCpuRegistersV1,
    RemoteCpuRegistersV2,
    RemoteCpuRegistersV3,
    RemoteCpuRegistersV3A,
    ServerCpuType,
};

#[derive(Clone, Default)]
pub struct RemoteCpuState {
    pub regs: RemoteCpuRegisters,
}

/// UI-owned cached state. This type deliberately has no serial-port handle.
pub struct ClientContext {
    pub(crate) port_name: String,
    pub(crate) baud_rate: Option<u32>,
    pub(crate) busy: bool,
    pub(crate) cpu_type: ServerCpuType,
    pub(crate) queue_status: bool,
    pub(crate) program_state: ProgramState,
    pub(crate) server_flags: u32,
    initial_state: RemoteCpuState,
}

impl ClientContext {
    pub fn new(
        port_name: String,
        cpu_type: ServerCpuType,
        queue_status: bool,
        program_state: ProgramState,
        server_flags: u32,
    ) -> Self {
        let regs = match cpu_type {
            ServerCpuType::Intel80386 => {
                RemoteCpuRegisters::V3(RemoteCpuRegistersV3::A(RemoteCpuRegistersV3A::default()))
            }
            ServerCpuType::Intel80286 => {
                let mut regs = RemoteCpuRegistersV2::default();
                regs.normalize_descriptors();
                regs.cs_desc.set_base_address(0xFF0000);
                RemoteCpuRegisters::V2(regs)
            }
            _ => RemoteCpuRegisters::V1(RemoteCpuRegistersV1::default()),
        };
        Self {
            port_name,
            baud_rate: None,
            busy: false,
            cpu_type,
            queue_status,
            program_state,
            server_flags,
            initial_state: RemoteCpuState { regs },
        }
    }

    pub fn control_state(&self) -> ClientControlState {
        ClientControlState::Setup
    }
    pub fn initial_state(&self) -> &RemoteCpuState {
        &self.initial_state
    }
    pub fn initial_state_mut(&mut self) -> &mut RemoteCpuState {
        &mut self.initial_state
    }
    pub fn set_initial_state(&mut self, initial_state: &RemoteCpuState) {
        self.initial_state = initial_state.clone();
    }
    pub fn cached_flags(&self) -> u32 {
        self.server_flags
    }
}
