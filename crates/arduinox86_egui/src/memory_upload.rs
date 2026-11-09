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

use anyhow::{anyhow, Result};
use arduinox86_client::{trap::InstructionAddress, RemoteCpuRegisters, ServerCpuType, ServerFlags};

use crate::disassembly::last_instruction_offset;

pub struct MemoryUpload {
    pub name:    String,
    pub address: u32,
    pub data:    Vec<u8>,
    pub is_program: bool,
}

impl MemoryUpload {
    pub(crate) fn trap_endpoint(&self, cpu: ServerCpuType, regs: &RemoteCpuRegisters, flags: u32) -> Result<Option<InstructionAddress>> {
        if !self.is_program || flags & ServerFlags::ENABLE_TRAP_EXECUTION == 0 {
            return Ok(None);
        }
        
        let RemoteCpuRegisters::V2(regs) = regs else {
            return Err(anyhow!("Trap program uploads require 286 registers"));
        };
        
        let last_byte = self.data.len().checked_sub(1)
            .ok_or_else(|| anyhow!("Program {} contains no instructions", self.name))?;
        let end_address = u32::try_from(last_byte).ok().and_then(|offset| self.address.checked_add(offset))
            .filter(|address| *address <= 0xFFFFFF)
            .ok_or_else(|| anyhow!("Program {} exceeds the 286 address bus", self.name))?;
        let base = regs.cs_desc.base_address();

        if self.address < base || end_address - base > u32::from(regs.cs_desc.limit()) {
            return Err(anyhow!("Program {} must fit within the initial CS segment", self.name));
        }
        
        let offset = last_instruction_offset(cpu, &self.data)
            .map_err(|error| anyhow!("Cannot select endpoint for {}: {error}", self.name))?;
        let endpoint_ip = self.address - base + offset as u32;
        
        Ok(Some(InstructionAddress { cs: regs.cs, ip: endpoint_ip as u16 }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn program_upload(address: u32, data: &[u8]) -> MemoryUpload {
        MemoryUpload { name: "test program".into(), address, data: data.to_vec(), is_program: true }
    }

    #[test]
    fn trap_endpoint_is_final_instruction() {
        let mut regs = arduinox86_client::RemoteCpuRegistersV2 {
            cs: 0x18, ip: 0x100, msw: 1, ..Default::default()
        };
        // This protected segment is deliberately unrelated to selector << 4.
        regs.cs_desc.set_base_address(0x12003);
        let regs = RemoteCpuRegisters::V2(regs);
        for (bytes, ip) in [
            (vec![0xB8, 0x34, 0x12], 0x100),
            (vec![0xB8, 0x34, 0x12, 0x90], 0x103),
            (vec![0x40, 0xB8, 0x34, 0x90], 0x101),
        ] {
            let upload = program_upload(0x12103, &bytes);
            let endpoint = upload.trap_endpoint(ServerCpuType::Intel80286, &regs, ServerFlags::ENABLE_TRAP_EXECUTION).unwrap();
            assert_eq!(endpoint, Some(InstructionAddress { cs: 0x18, ip }));
            assert_eq!(upload.data, bytes);
        }
        for bytes in [&[][..], &[0x40, 0xB8, 0x34][..], &[0x0F][..]] {
            let upload = program_upload(0x12203, bytes);
            assert!(upload.trap_endpoint(ServerCpuType::Intel80286, &regs, ServerFlags::ENABLE_TRAP_EXECUTION).is_err());
            assert_eq!(upload.data, bytes);
        }
    }

    #[test]
    fn upload_preserves_data_and_halt_code() {
        let regs = RemoteCpuRegisters::V2(Default::default());
        let mut upload = program_upload(0, &[0xB8, 0x34, 0x12]);
        assert_eq!(upload.trap_endpoint(ServerCpuType::Intel80286, &regs, 0).unwrap(), None);
        upload.is_program = false;
        assert_eq!(upload.trap_endpoint(ServerCpuType::Intel80286, &regs, ServerFlags::ENABLE_TRAP_EXECUTION).unwrap(), None);
        assert_eq!(upload.data, [0xB8, 0x34, 0x12]);
    }

    #[test]
    fn trap_endpoint_checks_address_limits() {
        let mut regs = arduinox86_client::RemoteCpuRegistersV2 { cs: 0x1000, ..Default::default() };
        regs.cs_desc.set_base_address(0x10000);
        let flags = ServerFlags::ENABLE_TRAP_EXECUTION;
        let ending_at_limit = program_upload(0x1FFFF, &[0x40]);
        assert_eq!(ending_at_limit.trap_endpoint(ServerCpuType::Intel80286, &RemoteCpuRegisters::V2(regs.clone()), flags).unwrap(),
            Some(InstructionAddress { cs: 0x1000, ip: 0xFFFF }));
        let final_word = program_upload(0x1FFFD, &[0xB8, 0x34, 0x12]);
        assert_eq!(final_word.trap_endpoint(ServerCpuType::Intel80286, &RemoteCpuRegisters::V2(regs.clone()), flags).unwrap(),
            Some(InstructionAddress { cs: 0x1000, ip: 0xFFFD }));
        for (address, limit) in [(0x1FFFF, 0xFFFF), (0x10010, 0x10), (0xFFFF, 0xFFFF)] {
            regs.cs_desc.set_limit(limit);
            let upload = program_upload(address, &[0xB8, 0x34, 0x12]);
            assert!(upload.trap_endpoint(ServerCpuType::Intel80286, &RemoteCpuRegisters::V2(regs.clone()), flags).is_err());
            assert_eq!(upload.data, [0xB8, 0x34, 0x12]);
        }
        regs.cs_desc.set_base_address(0xFFFFFF);
        regs.cs_desc.set_limit(0xFFFF);
        let regs = RemoteCpuRegisters::V2(regs);
        let upload = program_upload(0xFFFFFF, &[0x40]);
        assert_eq!(upload.trap_endpoint(ServerCpuType::Intel80286, &regs, flags).unwrap(),
            Some(InstructionAddress { cs: 0x1000, ip: 0 }));
        for address in [0xFFFFFF, u32::MAX] {
            assert!(program_upload(address, &[0xB8, 0x34, 0x12])
                .trap_endpoint(ServerCpuType::Intel80286, &regs, flags).is_err());
        }
    }
}
