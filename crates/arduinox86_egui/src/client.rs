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
use crate::serial_manager::SerialManager;
use arduinox86_client::{
    CpuClient,
    ProgramState,
    RemoteCpuRegisters,
    RemoteCpuRegistersV1,
    RemoteCpuRegistersV2,
    RemoteCpuRegistersV3,
    RemoteCpuRegistersV3A,
    ServerCpuType,
};

use crate::enums::ClientControlState;
use anyhow::Result;

#[derive(Clone, Default)]
pub struct RemoteCpuState {
    pub regs: RemoteCpuRegisters,
}

pub struct ClientContext {
    pub(crate) port_name: String,
    pub(crate) client_state: ClientControlState,
    pub(crate) client: CpuClient,
    pub(crate) cpu_type: ServerCpuType,
    server_flags: u32,
    pub(crate) queue_status: bool,
    pub(crate) program_state: ProgramState,

    pub(crate) initial_state: RemoteCpuState,
    pub(crate) memory_vec:    Vec<u8>,
}

impl ClientContext {
    pub fn new(selected_port: usize, sm: &mut SerialManager) -> Result<Self> {
        // Get port name from selection
        let selected_port = sm
            .ports()
            .get(selected_port)
            .ok_or_else(|| anyhow::anyhow!("Invalid port selection"))?;

        let port_name = selected_port.port_name.clone();
        let mut client = CpuClient::init(Some(port_name.clone()), None)?;
        let (cpu_type, queue_status) = client.cpu_type()?;

        // Create the appropriate register state type based on the CPU type.
        let initial_regs = match cpu_type {
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

        let initial_state = RemoteCpuState { regs: initial_regs };

        let program_state = client.get_program_state()?;
        let server_flags = client.get_flags()?;

        Ok(Self {
            port_name,
            client_state: ClientControlState::Setup,
            client,
            cpu_type,
            server_flags,
            queue_status,
            program_state,
            initial_state,
            memory_vec: Vec::with_capacity(u16::MAX as usize),
        })
    }

    pub fn control_state(&self) -> ClientControlState {
        self.client_state
    }

    pub fn program_state(&mut self) -> Result<ProgramState> {
        self.program_state = self.client.get_program_state()?;
        Ok(self.program_state)
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

    pub fn read_memory(&mut self, address: u32, size: u32) -> Result<&[u8]> {
        self.memory_vec.clear();
        let mut writer = std::io::Cursor::new(&mut self.memory_vec);
        self.client
            .read_memory(address, size, &mut writer)
            .map_err(|e| anyhow::anyhow!(e.to_string()))?;

        Ok(writer.into_inner())
    }

    pub fn erase_memory(&mut self) -> Result<()> {
        self.client.erase_memory().map_err(|e| anyhow::anyhow!(e.to_string()))?;
        Ok(())
    }

    pub fn set_flag_state(&mut self, flag: u32, state: bool) -> Result<bool> {
        let mut flags = self.client.get_flags()?;
        if state {
            flags |= flag;
        }
        else {
            flags &= !flag;
        }
        self.client.set_flags(flags)?;
        self.server_flags = flags;
        Ok(state)
    }

    pub fn cached_flag_state(&self, flag: u32) -> bool {
        self.server_flags & flag != 0
    }

    pub fn cached_flags(&self) -> u32 {
        self.server_flags
    }
}
