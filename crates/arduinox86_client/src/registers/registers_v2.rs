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

use std::io::Write;

#[cfg(feature = "use_iced")]
use crate::registers_common::RandomizeOpts;

use rand::Rng;
use rand_distr::{Beta, Distribution};

use binrw::{binrw, BinRead, BinReaderExt, BinWrite};
use modular_bitfield::{bitfield, prelude::*};

#[cfg(feature = "use_moo")]
use moo::prelude::*;

#[bitfield]
#[derive(Default, Debug)]
pub struct SegmentDescriptorAccessByte {
    pub d_type: B4,
    pub s: B1,
    pub dpl: B2,
    pub p: B1,
}

#[bitfield]
#[derive(BinRead, BinWrite)]
#[br(map = Self::from_bytes)]
#[bw(map = |&x| Self::into_bytes(x))]
#[derive(Copy, Clone, Debug)]
pub struct SegmentDescriptorV1 {
    pub base_address: B24,
    pub d_type: B4,
    pub s: B1,
    pub dpl: B2,
    pub p: B1,
    pub limit: B16,
}

impl Default for SegmentDescriptorV1 {
    fn default() -> Self {
        SegmentDescriptorV1::new()
            .with_base_address(0)
            .with_d_type(0x02)
            .with_s(0)
            .with_dpl(0)
            .with_p(1)
            .with_limit(0xFFFF)
    }
}

impl SegmentDescriptorV1 {
    pub fn to_buffer<W: Write>(&self, buffer: &mut W) -> std::io::Result<()> {
        let bytes = self.clone().into_bytes();
        buffer.write_all(&bytes)?;
        Ok(())
    }
}

/// [RemoteCpuRegistersV2] is the full set of registers for the Intel 80286.
/// This structure is loaded via the LOADALL instruction, 0F 05.
#[binrw]
#[brw(little)]
#[derive(Clone, Debug)]
pub struct RemoteCpuRegistersV2 {
    pub x0: u16,
    pub x1: u16,
    pub x2: u16,
    pub msw: u16,
    pub x3: u16,
    pub x4: u16,
    pub x5: u16,
    pub x6: u16,
    pub x7: u16,
    pub x8: u16,
    pub x9: u16,
    pub tr: u16,
    pub flags: u16,
    pub ip: u16,
    pub ldt: u16,
    pub ds: u16,
    pub ss: u16,
    pub cs: u16,
    pub es: u16,
    pub di: u16,
    pub si: u16,
    pub bp: u16,
    pub sp: u16,
    pub bx: u16,
    pub dx: u16,
    pub cx: u16,
    pub ax: u16,
    pub es_desc: SegmentDescriptorV1,
    pub cs_desc: SegmentDescriptorV1,
    pub ss_desc: SegmentDescriptorV1,
    pub ds_desc: SegmentDescriptorV1,
    pub gdt_desc: SegmentDescriptorV1,
    pub ldt_desc: SegmentDescriptorV1,
    pub idt_desc: SegmentDescriptorV1,
    pub tss_desc: SegmentDescriptorV1,
}

impl TryFrom<&[u8]> for RemoteCpuRegistersV2 {
    type Error = &'static str;
    fn try_from(buf: &[u8]) -> Result<Self, Self::Error> {
        parse_v2(&buf)
    }
}

impl From<[u8; 102]> for RemoteCpuRegistersV2 {
    fn from(buf: [u8; 102]) -> RemoteCpuRegistersV2 {
        parse_v2(&buf).unwrap()
    }
}

fn parse_v2(buf: &[u8]) -> Result<RemoteCpuRegistersV2, &'static str> {
    if buf.len() < 102 {
        return Err("Buffer too small for RemoteCpuRegistersV2");
    }

    let mut new_regs = RemoteCpuRegistersV2::default();
    let mut cursor = std::io::Cursor::new(buf);

    new_regs.x0 = cursor.read_le().unwrap(); // 800
    new_regs.x1 = cursor.read_le().unwrap(); // 802
    new_regs.x2 = cursor.read_le().unwrap(); // 804

    new_regs.msw = cursor.read_le().unwrap(); // 806

    new_regs.x3 = cursor.read_le().unwrap(); // 808
    new_regs.x4 = cursor.read_le().unwrap(); // 80A
    new_regs.x5 = cursor.read_le().unwrap(); // 80C
    new_regs.x6 = cursor.read_le().unwrap(); // 80E
    new_regs.x7 = cursor.read_le().unwrap(); // 810
    new_regs.x8 = cursor.read_le().unwrap(); // 812
    new_regs.x9 = cursor.read_le().unwrap(); // 814

    new_regs.tr = cursor.read_le().unwrap(); // 816
    new_regs.flags = cursor.read_le().unwrap(); // 818
    new_regs.ip = cursor.read_le().unwrap(); // 81A
    new_regs.ldt = cursor.read_le().unwrap(); // 81C

    new_regs.ds = cursor.read_le().unwrap(); // 81E
    new_regs.ss = cursor.read_le().unwrap(); // 820
    new_regs.cs = cursor.read_le().unwrap(); // 822
    new_regs.es = cursor.read_le().unwrap(); // 824

    new_regs.di = cursor.read_le().unwrap(); // 826
    new_regs.si = cursor.read_le().unwrap(); // 828
    new_regs.bp = cursor.read_le().unwrap(); // 82A
    new_regs.sp = cursor.read_le().unwrap(); // 82C

    new_regs.bx = cursor.read_le().unwrap(); // 82E
    new_regs.dx = cursor.read_le().unwrap(); // 830
    new_regs.cx = cursor.read_le().unwrap(); // 832
    new_regs.ax = cursor.read_le().unwrap(); // 834

    let idx = cursor.position();
    let desc_slice = &cursor.into_inner()[idx as usize..idx as usize + 48];

    new_regs.es_desc = read_descriptor(desc_slice, 0);
    new_regs.cs_desc = read_descriptor(desc_slice, 1);
    new_regs.ss_desc = read_descriptor(desc_slice, 2);
    new_regs.ds_desc = read_descriptor(desc_slice, 3);
    new_regs.gdt_desc = read_descriptor(desc_slice, 4);
    new_regs.ldt_desc = read_descriptor(desc_slice, 5);
    new_regs.idt_desc = read_descriptor(desc_slice, 6);
    new_regs.tss_desc = read_descriptor(desc_slice, 7);

    Ok(new_regs)
}

fn read_descriptor(slice: &[u8], index: usize) -> SegmentDescriptorV1 {
    // each descriptor is 6 bytes
    let start = index * 6;
    let end = start + 6;
    let bytes: [u8; 6] = slice[start..end]
        .try_into()
        .expect("desc_slice must be at least 6*8=48 bytes");
    SegmentDescriptorV1::from_bytes(bytes)
}

impl Default for RemoteCpuRegistersV2 {
    fn default() -> Self {
        // Match the Harris post-power-on STOREALL access bytes: FF for GDT/IDT/TSS, 7F for LDT.
        // https://gist.github.com/dbalsom/8c7900adbf4d1405fc5ce5d5a521b0e9
        let system_descriptor = SegmentDescriptorV1::default().with_d_type(0xF).with_s(1).with_dpl(3);
        RemoteCpuRegistersV2 {
            x0: 0,
            x1: 0,
            x2: 0x002A,  // X2 is always 2A (42).
            msw: 0xFFF0, // MSW is always 0xFFF0 on reset.
            x3: 0,
            x4: 0,
            x5: 0,
            x6: 0,
            x7: 0,
            x8: 0,
            x9: 0,
            tr: 0,
            flags: 0x0002, // Flags are always 0x0002 on reset (bit 1 is reserved and always 1)
            ip: 0xFFF0,    // IP is always 0xFFF0 on reset (reset vector is F000:FFF0).
            ldt: 0,
            ds: 0,
            ss: 0,
            cs: 0xF000, // CS is always 0xF000 on reset (reset vector is F000:FFF0).
            es: 0,
            di: 0,
            si: 0,
            bp: 0,
            sp: 0,
            bx: 0,
            dx: 0,
            cx: 0,
            ax: 0,
            es_desc: SegmentDescriptorV1::default(),
            cs_desc: SegmentDescriptorV1::default(),
            ss_desc: SegmentDescriptorV1::default(),
            ds_desc: SegmentDescriptorV1::default(),
            gdt_desc: system_descriptor,
            ldt_desc: system_descriptor.with_p(0),
            idt_desc: system_descriptor,
            tss_desc: system_descriptor,
        }
    }
}

impl RemoteCpuRegistersV2 {
    pub const FLAGS_RESERVED_SET: u16 = 0x0002; // Reserved bit in flags register, always set to 1.
    pub const FLAGS_RESERVED_MASK: u16 = 0x7FD7; // Reserved bits 3, 5, and 15 are always cleared to 0.

    pub fn to_buffer<W: Write>(&self, buffer: &mut W) {
        buffer.write_all(&self.x0.to_le_bytes()).unwrap();
        buffer.write_all(&self.x1.to_le_bytes()).unwrap();
        buffer.write_all(&self.x2.to_le_bytes()).unwrap();
        buffer.write_all(&self.msw.to_le_bytes()).unwrap();
        buffer.write_all(&self.x3.to_le_bytes()).unwrap();
        buffer.write_all(&self.x4.to_le_bytes()).unwrap();
        buffer.write_all(&self.x5.to_le_bytes()).unwrap();
        buffer.write_all(&self.x6.to_le_bytes()).unwrap();
        buffer.write_all(&self.x7.to_le_bytes()).unwrap();
        buffer.write_all(&self.x8.to_le_bytes()).unwrap();
        buffer.write_all(&self.x9.to_le_bytes()).unwrap();
        buffer.write_all(&self.tr.to_le_bytes()).unwrap();
        buffer.write_all(&self.flags.to_le_bytes()).unwrap();
        buffer.write_all(&self.ip.to_le_bytes()).unwrap();
        buffer.write_all(&self.ldt.to_le_bytes()).unwrap();
        buffer.write_all(&self.ds.to_le_bytes()).unwrap();
        buffer.write_all(&self.ss.to_le_bytes()).unwrap();
        buffer.write_all(&self.cs.to_le_bytes()).unwrap();
        buffer.write_all(&self.es.to_le_bytes()).unwrap();
        buffer.write_all(&self.di.to_le_bytes()).unwrap();
        buffer.write_all(&self.si.to_le_bytes()).unwrap();
        buffer.write_all(&self.bp.to_le_bytes()).unwrap();
        buffer.write_all(&self.sp.to_le_bytes()).unwrap();
        buffer.write_all(&self.bx.to_le_bytes()).unwrap();
        buffer.write_all(&self.dx.to_le_bytes()).unwrap();
        buffer.write_all(&self.cx.to_le_bytes()).unwrap();
        buffer.write_all(&self.ax.to_le_bytes()).unwrap();

        // Write segment descriptors
        self.es_desc.to_buffer(buffer).expect("Failed to write es_desc");
        self.cs_desc.to_buffer(buffer).expect("Failed to write cs_desc");
        self.ss_desc.to_buffer(buffer).expect("Failed to write ss_desc");
        self.ds_desc.to_buffer(buffer).expect("Failed to write ds_desc");
        self.gdt_desc.to_buffer(buffer).expect("Failed to write gdt_desc");
        self.ldt_desc.to_buffer(buffer).expect("Failed to write ldt_desc");
        self.idt_desc.to_buffer(buffer).expect("Failed to write idt_desc");
        self.tss_desc.to_buffer(buffer).expect("Failed to write tss_desc");
    }

    pub fn rewind_ip(&mut self, adjust: u16) {
        self.ip = self.ip.wrapping_sub(adjust);
    }

    pub fn clear_trap_flag(&mut self) {
        // Clear the trap flag (bit 8) in the flags register.
        self.flags &= !0x0100; // Clear bit 8
    }
    pub fn clear_interrupt_flag(&mut self) {
        // Clear the interrupt flag (bit 9) in the flags register.
        self.flags &= !0x0200; // Clear bit 9
    }

    /// Initialize segment descriptors with a base address calculated from the actual segment
    /// value, as would be the case in real mode.
    pub fn normalize_descriptors(&mut self) {
        let es_base = (self.es as u32) << 4;
        let cs_base = (self.cs as u32) << 4;
        let ss_base = (self.ss as u32) << 4;
        let ds_base = (self.ds as u32) << 4;

        log::trace!("Using CS base of {:06X}", cs_base);
        self.es_desc = SegmentDescriptorV1::default()
            .with_base_address(es_base)
            .with_limit(0xFFFF);

        self.cs_desc = SegmentDescriptorV1::default()
            .with_base_address(cs_base)
            .with_limit(0xFFFF);

        self.ss_desc = SegmentDescriptorV1::default()
            .with_base_address(ss_base)
            .with_limit(0xFFFF);

        self.ds_desc = SegmentDescriptorV1::default()
            .with_base_address(ds_base)
            .with_limit(0xFFFF);
    }

    pub fn weighted_u16(
        weight_zero: f32,
        weight_ones: f32,
        rand: &mut rand::rngs::StdRng,
        register_beta: &mut Beta<f64>,
    ) -> u16 {
        let random_value: f32 = rand.random();
        if random_value < weight_zero {
            0
        }
        else if random_value < weight_zero + weight_ones {
            0xFFFF // All bits set to 1
        }
        else {
            let value: u16 = (register_beta.sample(rand) * u16::MAX as f64) as u16;
            value
        }
    }

    #[cfg(feature = "use_iced")]
    #[rustfmt::skip]
    pub fn randomize(
        &mut self,
        opts: &RandomizeOpts,
        rand: &mut rand::rngs::StdRng,
        beta: &mut Beta<f64>,
        weighted_index: &rand::distr::weighted::WeightedIndex<f32>,
        inject_values: &[u32])
    {
        *self = RemoteCpuRegistersV2::default(); // Reset all registers to default values

        if opts.randomize_flags {
            self.flags = (rand.random::<u16>() | RemoteCpuRegistersV2::FLAGS_RESERVED_SET) & RemoteCpuRegistersV2::FLAGS_RESERVED_MASK; // Set reserved bit
        }
        if opts.clear_trap_flag {
            self.clear_trap_flag();
        }
        if opts.clear_interrupt_flag {
            self.clear_interrupt_flag();
        }

        if opts.randomize_general {
            self.ax = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.bx = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.cx = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.dx = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.sp = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.bp = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.si = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.di = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.ds = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.ss = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.es = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
            self.cs = RemoteCpuRegistersV2::weighted_u16(opts.weight_zero, opts.weight_ones, rand, beta);
        }

        if opts.randomize_ip {
            self.ip = rand.random::<u16>() & opts.ip_mask;
        }

        // Set SP to even value.
        self.sp = self.sp & !1;
        // Use the sp_odd_chance to set it to an odd value based on the configured percentage.
        let odd_chance = rand.random::<f32>();
        if odd_chance < opts.weight_sp_odd {
            self.sp |= 1; // Set the least significant bit to 1 to make it odd
        }

        // Set sp to minimum value if beneath.
        if self.sp < opts.sp_min_value as u16 {
            self.sp = opts.sp_min_value as u16;
        }

        // Set sp to maximum value if above.
        if self.sp > opts.sp_max_value as u16 {
            self.sp = opts.sp_max_value as u16;
        }

        if opts.sp_use_ss_limit {
            let limit = self.ss_desc.limit();
            self.sp &= limit;
        }

        if opts.randomize_x {
            self.x0 = rand.random();
            self.x1 = rand.random();
            //self.x2 = rand.random(); // Cant set X2.
            self.x3 = rand.random();
            self.x4 = rand.random();
            self.x5 = rand.random();
            self.x6 = rand.random();
            self.x7 = rand.random();
            self.x8 = rand.random();
            self.x9 = rand.random();
        }

        if opts.randomize_msw {
            self.msw = rand.random::<u16>() & 0xFFF0; // Keep reserved bits
        }

        if opts.randomize_tr {
            self.tr = rand.random::<u16>();
        }

        if opts.randomize_ldt {
            self.ldt = rand.random::<u16>();
        }

        if opts.randomize_segment_descriptors {
            let base_address: u32 = (rand.random::<u16>() as u32) << 4;
            let limit: u16 = 0xFFFF;

            // Randomize segment descriptors
            self.es_desc = SegmentDescriptorV1::default()
                .with_base_address(base_address)
                .with_limit(limit);
            self.cs_desc = SegmentDescriptorV1::default()
                .with_base_address(base_address)
                .with_limit(limit);
            self.ss_desc = SegmentDescriptorV1::default()
                .with_base_address(base_address)
                .with_limit(limit);
            self.ds_desc = SegmentDescriptorV1::default()
                .with_base_address(base_address)
                .with_limit(limit);
        }
    }

    pub fn calculate_code_address(&self) -> u32 {
        // Calculate the code address based on CS descriptor base and IP
        self.cs_desc.base_address() + (self.ip as u32)
    }
}

#[cfg(feature = "use_moo")]
impl From<RemoteCpuRegistersV2> for MooRegisters16 {
    fn from(remote: RemoteCpuRegistersV2) -> Self {
        MooRegisters16::from(&remote)
    }
}

#[cfg(feature = "use_moo")]
impl From<&RemoteCpuRegistersV2> for MooRegisters16 {
    fn from(remote: &RemoteCpuRegistersV2) -> Self {
        (&MooRegisters16Init {
            ax:    remote.ax,
            bx:    remote.bx,
            cx:    remote.cx,
            dx:    remote.dx,
            cs:    remote.cs,
            ss:    remote.ss,
            ds:    remote.ds,
            es:    remote.es,
            sp:    remote.sp,
            bp:    remote.bp,
            si:    remote.si,
            di:    remote.di,
            ip:    remote.ip,
            flags: remote.flags,
        })
            .into()
    }
}
