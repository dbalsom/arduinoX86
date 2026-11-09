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

use crate::registers::register_traits::Registers32;
use binrw::BinWrite;
use std::io::{Seek, Write};

// #[cfg(feature = "use_moo")]
// use moo::prelude::MooRegisters16Init;
// #[cfg(feature = "use_moo")]
// use moo::types::MooRegisters16;

use crate::{
    RemoteCpuRegistersV1,
    RemoteCpuRegistersV2,
    RemoteCpuRegistersV3,
    RemoteCpuRegistersV3A,
    RemoteCpuRegistersV3B,
};

#[derive(Clone, Debug)]
pub enum RemoteCpuRegisters {
    V1(RemoteCpuRegistersV1),
    V2(RemoteCpuRegistersV2),
    V3(RemoteCpuRegistersV3),
}

impl TryFrom<&[u8]> for RemoteCpuRegisters {
    type Error = &'static str;

    fn try_from(buf: &[u8]) -> Result<Self, Self::Error> {
        if buf.len() == 28 {
            Ok(RemoteCpuRegisters::V1(RemoteCpuRegistersV1::from(buf)))
        }
        else if buf.len() == 102 {
            Ok(RemoteCpuRegisters::V2(RemoteCpuRegistersV2::try_from(buf)?))
        }
        else if buf.len() == 204 {
            Ok(RemoteCpuRegisters::V3(RemoteCpuRegistersV3::A(
                RemoteCpuRegistersV3A::try_from(buf)?,
            )))
        }
        else if buf.len() == 208 {
            Ok(RemoteCpuRegisters::V3(RemoteCpuRegistersV3::B(
                RemoteCpuRegistersV3B::try_from(buf)?,
            )))
        }
        else {
            log::error!("Invalid buffer length for RemoteCpuRegisters: {}", buf.len());
            Err("Invalid buffer length for RemoteCpuRegisters!")
        }
    }
}

impl Default for RemoteCpuRegisters {
    fn default() -> Self {
        RemoteCpuRegisters::V1(RemoteCpuRegistersV1::default())
    }
}

impl RemoteCpuRegisters {
    pub fn to_b(&self) -> Option<RemoteCpuRegisters> {
        match self {
            RemoteCpuRegisters::V3(RemoteCpuRegistersV3::A(regs_a)) => Some(RemoteCpuRegisters::V3(
                RemoteCpuRegistersV3::B(RemoteCpuRegistersV3B::from(regs_a)),
            )),
            RemoteCpuRegisters::V3(RemoteCpuRegistersV3::B(_)) => Some(self.clone()),
            _ => None,
        }
    }

    pub fn set_cs(&mut self, cs: u16) {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.cs = cs,
            RemoteCpuRegisters::V2(regs) => regs.cs = cs,
            RemoteCpuRegisters::V3(regs) => regs.set_cs(cs),
        }
    }

    pub fn set_ip(&mut self, ip: u16) {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.ip = ip,
            RemoteCpuRegisters::V2(regs) => regs.ip = ip,
            RemoteCpuRegisters::V3(_) => {}
        }
    }

    pub fn rewind_ip(&mut self, adjust: u16) {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.rewind_ip(adjust),
            RemoteCpuRegisters::V2(regs) => regs.rewind_ip(adjust),
            RemoteCpuRegisters::V3(_) => {}
        }
    }

    pub fn ax(&self) -> u16 {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.ax,
            RemoteCpuRegisters::V2(regs) => regs.ax,
            RemoteCpuRegisters::V3(regs) => 0,
        }
    }

    pub fn flags(&self) -> u16 {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.flags,
            RemoteCpuRegisters::V2(regs) => regs.flags,
            RemoteCpuRegisters::V3(_) => 0,
        }
    }

    pub fn code_address(&self) -> u32 {
        match self {
            RemoteCpuRegisters::V1(regs) => regs.calculate_code_address(),
            RemoteCpuRegisters::V2(regs) => regs.calculate_code_address(),
            RemoteCpuRegisters::V3(regs) => regs.calculate_code_address(),
        }
    }

    pub fn normalize(&mut self) {
        match self {
            RemoteCpuRegisters::V1(regs) => {}
            RemoteCpuRegisters::V2(regs) => regs.normalize_descriptors(),
            RemoteCpuRegisters::V3(regs) => regs.normalize_descriptors(),
        }
    }

    pub fn write<WS: Write + Seek>(&self, writer: &mut WS) -> std::io::Result<()> {
        let mut buf = vec![0u8; 204];

        match self {
            RemoteCpuRegisters::V1(regs) => {
                regs.write_buf(&mut buf);
                writer.write_all(&buf[0..28])
            }
            RemoteCpuRegisters::V2(regs) => regs.write_le(writer).map_err(|e| {
                std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("Failed to write RemoteCpuRegistersV2: {}", e),
                )
            }),
            RemoteCpuRegisters::V3(RemoteCpuRegistersV3::A(regs)) => regs.write_le(writer).map_err(|e| {
                std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("Failed to write RemoteCpuRegistersV3A: {}", e),
                )
            }),
            RemoteCpuRegisters::V3(RemoteCpuRegistersV3::B(regs)) => regs.write_le(writer).map_err(|e| {
                std::io::Error::new(
                    std::io::ErrorKind::Other,
                    format!("Failed to write RemoteCpuRegistersV3B: {}", e),
                )
            }),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SegmentSize {
    Sixteen,
    ThirtyTwo,
}

impl From<SegmentSize> for u32 {
    fn from(size: SegmentSize) -> Self {
        match size {
            SegmentSize::Sixteen => 16,
            SegmentSize::ThirtyTwo => 32,
        }
    }
}

#[derive(Clone, Default)]
pub struct RandomizeOpts {
    pub real_mode: bool,
    pub weight_zero: f32,
    pub weight_ones: f32,
    pub weight_inject: f32,
    pub weight_sp_odd: f32,
    pub sp_min_value: u32,
    pub sp_max_value: u32,
    pub sp_use_ss_limit: bool,
    pub randomize_flags: bool,
    pub clear_trap_flag: bool,
    pub clear_interrupt_flag: bool,
    pub clear_resume_flag: bool,
    pub randomize_general: bool,
    pub randomize_ip: bool,
    pub ip_mask: u16,
    pub eip_mask: u32,
    pub randomize_x: bool,
    pub randomize_msw: bool,
    pub randomize_tr: bool,
    pub randomize_ldt: bool,
    pub randomize_segment_descriptors: bool,
    pub randomize_table_descriptors: bool,

    pub mask_eac_registers: bool,
}
