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

use crate::TEXT_COLOR;

use arduinox86_client::{Registers32, RemoteCpuRegistersV3, ServerCpuType};
use egui::Color32;

#[derive(Debug, Clone)]
pub struct RegisterString {
    pub text:    String,
    pub color32: Color32,
}

impl RegisterString {
    fn from<T: AsRef<str>>(text: T) -> Self {
        RegisterString {
            text:    text.as_ref().to_string(),
            color32: TEXT_COLOR,
        }
    }

    pub(crate) fn from_diff<T: AsRef<str>>(text: T, diff: bool) -> Self {
        RegisterString {
            text:    text.as_ref().to_string(),
            color32: if diff { Color32::CYAN } else { TEXT_COLOR },
        }
    }
}

impl Default for RegisterString {
    fn default() -> Self {
        RegisterString {
            text:    "0".to_string(),
            color32: TEXT_COLOR,
        }
    }
}

impl RegisterString {
    #[inline]
    pub fn text(&self) -> &str {
        &self.text
    }
}

#[derive(Debug, Clone)]
pub struct RegisterStringStateV3 {
    pub cr0: RegisterString,
    pub eflags_raw: RegisterString,
    pub eip: RegisterString,
    pub edi: RegisterString,
    pub esi: RegisterString,
    pub ebp: RegisterString,
    pub esp: RegisterString,
    pub ebx: RegisterString,
    pub edx: RegisterString,
    pub ecx: RegisterString,
    pub eax: RegisterString,
    pub dr6: RegisterString,
    pub dr7: RegisterString,
    pub gs: RegisterString,
    pub fs: RegisterString,
    pub ds: RegisterString,
    pub ss: RegisterString,
    pub cs: RegisterString,
    pub es: RegisterString,
    pub flags: FlagStringState,
}

impl RegisterStringStateV3 {
    pub fn from_delta_v3(initial_regs: &RemoteCpuRegistersV3, final_regs: &RemoteCpuRegistersV3) -> Self {
        let cr0_diff = initial_regs.cr0() != final_regs.cr0();
        let eflags_raw_diff = initial_regs.eflags() != final_regs.eflags();
        let eip_diff = initial_regs.eip() != final_regs.eip();
        let edi_diff = initial_regs.edi() != final_regs.edi();
        let esi_diff = initial_regs.esi() != final_regs.esi();
        let ebp_diff = initial_regs.ebp() != final_regs.ebp();
        let esp_diff = initial_regs.esp() != final_regs.esp();
        let ebx_diff = initial_regs.ebx() != final_regs.ebx();
        let edx_diff = initial_regs.edx() != final_regs.edx();
        let ecx_diff = initial_regs.ecx() != final_regs.ecx();
        let eax_diff = initial_regs.eax() != final_regs.eax();
        let dr6_diff = initial_regs.dr6() != final_regs.dr6();
        let dr7_diff = initial_regs.dr7() != final_regs.dr7();
        let gs_diff = initial_regs.gs() != final_regs.gs();
        let fs_diff = initial_regs.fs() != final_regs.fs();
        let ds_diff = initial_regs.ds() != final_regs.ds();
        let ss_diff = initial_regs.ss() != final_regs.ss();
        let cs_diff = initial_regs.cs() != final_regs.cs();
        let es_diff = initial_regs.es() != final_regs.es();

        let new_strings = Self {
            cr0: RegisterString::from_diff(format!("{:#08x}", final_regs.cr0()), cr0_diff),
            eflags_raw: RegisterString::from_diff(format!("{:#08x}", final_regs.eflags()), eflags_raw_diff),
            eip: RegisterString::from_diff(format!("{:08x}", final_regs.eip()), eip_diff),
            edi: RegisterString::from_diff(format!("{:08x}", final_regs.edi()), edi_diff),
            esi: RegisterString::from_diff(format!("{:08x}", final_regs.esi()), esi_diff),
            ebp: RegisterString::from_diff(format!("{:08x}", final_regs.ebp()), ebp_diff),
            esp: RegisterString::from_diff(format!("{:08x}", final_regs.esp()), esp_diff),
            ebx: RegisterString::from_diff(format!("{:08x}", final_regs.ebx()), ebx_diff),
            edx: RegisterString::from_diff(format!("{:08x}", final_regs.edx()), edx_diff),
            ecx: RegisterString::from_diff(format!("{:08x}", final_regs.ecx()), ecx_diff),
            eax: RegisterString::from_diff(format!("{:08x}", final_regs.eax()), eax_diff),
            dr6: RegisterString::from_diff(format!("{:08x}", final_regs.dr6()), dr6_diff),
            dr7: RegisterString::from_diff(format!("{:08x}", final_regs.dr7()), dr7_diff),
            gs: RegisterString::from_diff(format!("{:04x}", final_regs.gs()), gs_diff),
            fs: RegisterString::from_diff(format!("{:04x}", final_regs.fs()), fs_diff),
            ds: RegisterString::from_diff(format!("{:04x}", final_regs.ds()), ds_diff),
            ss: RegisterString::from_diff(format!("{:04x}", final_regs.ss()), ss_diff),
            cs: RegisterString::from_diff(format!("{:04x}", final_regs.cs()), cs_diff),
            es: RegisterString::from_diff(format!("{:04x}", final_regs.es()), es_diff),
            flags: FlagStringState::from_diff(initial_regs.eflags(), final_regs.eflags(), ServerCpuType::Intel80386),
        };

        new_strings
    }
}

impl Default for RegisterStringStateV3 {
    fn default() -> Self {
        RegisterStringStateV3 {
            cr0: Default::default(),
            eflags_raw: Default::default(),
            eip: Default::default(),
            edi: Default::default(),
            esi: Default::default(),
            ebp: Default::default(),
            esp: Default::default(),
            ebx: Default::default(),
            edx: Default::default(),
            ecx: Default::default(),
            eax: Default::default(),
            dr6: Default::default(),
            dr7: Default::default(),
            gs: Default::default(),
            fs: Default::default(),
            ds: Default::default(),
            ss: Default::default(),
            cs: Default::default(),
            es: Default::default(),
            flags: FlagStringState::default(),
        }
    }
}

impl From<&RemoteCpuRegistersV3> for RegisterStringStateV3 {
    fn from(regs: &RemoteCpuRegistersV3) -> Self {
        RegisterStringStateV3 {
            cr0: RegisterString::from(format!("{:#08x}", regs.cr0())),
            eflags_raw: RegisterString::from(format!("{:#08x}", regs.eflags())),
            eip: RegisterString::from(format!("{:08x}", regs.eip())),
            edi: RegisterString::from(format!("{:08x}", regs.edi())),
            esi: RegisterString::from(format!("{:08x}", regs.esi())),
            ebp: RegisterString::from(format!("{:08x}", regs.ebp())),
            esp: RegisterString::from(format!("{:08x}", regs.esp())),
            ebx: RegisterString::from(format!("{:08x}", regs.ebx())),
            edx: RegisterString::from(format!("{:08x}", regs.edx())),
            ecx: RegisterString::from(format!("{:08x}", regs.ecx())),
            eax: RegisterString::from(format!("{:08x}", regs.eax())),
            dr6: RegisterString::from(format!("{:08x}", regs.dr6())),
            dr7: RegisterString::from(format!("{:08x}", regs.dr7())),
            gs: RegisterString::from(format!("{:04x}", regs.gs())),
            fs: RegisterString::from(format!("{:04x}", regs.fs())),
            ds: RegisterString::from(format!("{:04x}", regs.ds())),
            ss: RegisterString::from(format!("{:04x}", regs.ss())),
            cs: RegisterString::from(format!("{:04x}", regs.cs())),
            es: RegisterString::from(format!("{:04x}", regs.es())),
            flags: FlagStringState::new(regs.eflags(), ServerCpuType::Intel80386),
        }
    }
}

#[derive(Debug, Clone)]
pub struct FlagStringState {
    pub c_fl: RegisterString,
    pub p_fl: RegisterString,
    pub a_fl: RegisterString,
    pub z_fl: RegisterString,
    pub s_fl: RegisterString,
    pub t_fl: RegisterString,
    pub i_fl: RegisterString,
    pub d_fl: RegisterString,
    pub o_fl: RegisterString,
    pub m_fl: RegisterString,
}

impl Default for FlagStringState {
    fn default() -> Self {
        FlagStringState {
            c_fl: RegisterString::default(),
            p_fl: RegisterString::default(),
            a_fl: RegisterString::default(),
            z_fl: RegisterString::default(),
            s_fl: RegisterString::default(),
            t_fl: RegisterString::default(),
            i_fl: RegisterString::default(),
            d_fl: RegisterString::default(),
            o_fl: RegisterString::default(),
            m_fl: RegisterString::default(),
        }
    }
}

impl FlagStringState {
    pub const FLAG_CARRY: u32 = 0b0000_0000_0000_0001;
    pub const FLAG_RESERVED1: u32 = 0b0000_0000_0000_0010;
    pub const FLAG_PARITY: u32 = 0b0000_0000_0000_0100;
    pub const FLAG_RESERVED3: u32 = 0b0000_0000_0000_1000;
    pub const FLAG_AUX_CARRY: u32 = 0b0000_0000_0001_0000;
    pub const FLAG_RESERVED5: u32 = 0b0000_0000_0010_0000;
    pub const FLAG_ZERO: u32 = 0b0000_0000_0100_0000;
    pub const FLAG_SIGN: u32 = 0b0000_0000_1000_0000;
    pub const FLAG_TRAP: u32 = 0b0000_0001_0000_0000;
    pub const FLAG_INT_ENABLE: u32 = 0b0000_0010_0000_0000;
    pub const FLAG_DIRECTION: u32 = 0b0000_0100_0000_0000;
    pub const FLAG_OVERFLOW: u32 = 0b0000_1000_0000_0000;

    pub fn new(flags_raw: u32, _cpu_type: ServerCpuType) -> Self {
        FlagStringState {
            c_fl: {
                let fl = flags_raw & Self::FLAG_CARRY > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            p_fl: {
                let fl = flags_raw & Self::FLAG_PARITY > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            a_fl: {
                let fl = flags_raw & Self::FLAG_AUX_CARRY > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            z_fl: {
                let fl = flags_raw & Self::FLAG_ZERO > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            s_fl: {
                let fl = flags_raw & Self::FLAG_SIGN > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            t_fl: {
                let fl = flags_raw & Self::FLAG_TRAP > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            i_fl: {
                let fl = flags_raw & Self::FLAG_INT_ENABLE > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            d_fl: {
                let fl = flags_raw & Self::FLAG_DIRECTION > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            o_fl: {
                let fl = flags_raw & Self::FLAG_OVERFLOW > 0;
                RegisterString::from(format!("{:1}", fl as u8))
            },
            m_fl: { RegisterString::from("1".to_string()) },
        }
    }

    pub fn from_diff(initial_flags_raw: u32, final_flags_raw: u32, _cpu_type: ServerCpuType) -> Self {
        let c_fl_diff = (initial_flags_raw & Self::FLAG_CARRY) != (final_flags_raw & Self::FLAG_CARRY);
        let p_fl_diff = (initial_flags_raw & Self::FLAG_PARITY) != (final_flags_raw & Self::FLAG_PARITY);
        let a_fl_diff = (initial_flags_raw & Self::FLAG_AUX_CARRY) != (final_flags_raw & Self::FLAG_AUX_CARRY);
        let z_fl_diff = (initial_flags_raw & Self::FLAG_ZERO) != (final_flags_raw & Self::FLAG_ZERO);
        let s_fl_diff = (initial_flags_raw & Self::FLAG_SIGN) != (final_flags_raw & Self::FLAG_SIGN);
        let t_fl_diff = (initial_flags_raw & Self::FLAG_TRAP) != (final_flags_raw & Self::FLAG_TRAP);
        let i_fl_diff = (initial_flags_raw & Self::FLAG_INT_ENABLE) != (final_flags_raw & Self::FLAG_INT_ENABLE);
        let d_fl_diff = (initial_flags_raw & Self::FLAG_DIRECTION) != (final_flags_raw & Self::FLAG_DIRECTION);
        let o_fl_diff = (initial_flags_raw & Self::FLAG_OVERFLOW) != (final_flags_raw & Self::FLAG_OVERFLOW);

        FlagStringState {
            c_fl: {
                let fl = final_flags_raw & Self::FLAG_CARRY > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), c_fl_diff)
            },
            p_fl: {
                let fl = final_flags_raw & Self::FLAG_PARITY > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), p_fl_diff)
            },
            a_fl: {
                let fl = final_flags_raw & Self::FLAG_AUX_CARRY > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), a_fl_diff)
            },
            z_fl: {
                let fl = final_flags_raw & Self::FLAG_ZERO > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), z_fl_diff)
            },
            s_fl: {
                let fl = final_flags_raw & Self::FLAG_SIGN > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), s_fl_diff)
            },
            t_fl: {
                let fl = final_flags_raw & Self::FLAG_TRAP > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), t_fl_diff)
            },
            i_fl: {
                let fl = final_flags_raw & Self::FLAG_INT_ENABLE > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), i_fl_diff)
            },
            d_fl: {
                let fl = final_flags_raw & Self::FLAG_DIRECTION > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), d_fl_diff)
            },
            o_fl: {
                let fl = final_flags_raw & Self::FLAG_OVERFLOW > 0;
                RegisterString::from_diff(format!("{:1}", fl as u8), o_fl_diff)
            },
            m_fl: { RegisterString::from("1".to_string()) },
        }
    }
}
