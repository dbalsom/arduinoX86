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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ExecutionMode {
    QueueTracked,
    Trap,
    Halt,
}

impl ExecutionMode {
    /// Apply this mode's initial TF and managed IDTR settings, including to imported registers.
    pub(crate) fn apply_to_registers(self, regs: &mut arduinox86_client::RemoteCpuRegisters) {
        use arduinox86_client::{
            trap::{configure_managed_trap_registers, TRAP_FLAG},
            Registers32,
            RemoteCpuRegisters,
        };

        let tf = if self == Self::Trap { TRAP_FLAG } else { 0 };
        match regs {
            RemoteCpuRegisters::V1(regs) => regs.flags = (regs.flags & !TRAP_FLAG) | tf,
            RemoteCpuRegisters::V2(regs) => {
                regs.flags = (regs.flags & !TRAP_FLAG) | tf;
                if tf != 0 {
                    configure_managed_trap_registers(regs);
                }
            }
            RemoteCpuRegisters::V3(regs) => regs.set_eflags((regs.eflags() & !u32::from(TRAP_FLAG)) | u32::from(tf)),
        }
    }

    pub fn from_flags(flags: u32) -> Self {
        if flags & arduinox86_client::ServerFlags::ENABLE_TRAP_EXECUTION != 0 {
            Self::Trap
        }
        else {
            Self::Halt
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::QueueTracked => "Queue Tracked IP Termination (8088/8086 default)",
            Self::Trap => "Trap execution",
            Self::Halt => "Halt-based termination",
        }
    }

    pub fn unavailable_reason(self, cpu: arduinox86_client::ServerCpuType, flags: u32) -> Option<&'static str> {
        use arduinox86_client::{ServerCpuType, ServerFlags};

        match self {
            Self::QueueTracked => Some("Queue-tracked termination is not yet available in the GUI runner."),
            Self::Trap if !matches!(cpu, ServerCpuType::Intel80286) => Some("Trap execution requires an 80286."),
            Self::Trap if flags & (ServerFlags::USE_SMM | ServerFlags::EMU_8080) != 0 => {
                Some("Disable SMM and 8080 emulation before selecting trap execution.")
            }
            _ => None,
        }
    }
}

#[derive(strum_macros::Display, Copy, Clone, Debug, PartialEq, Eq)]
pub enum CpuStateType {
    Initial,
    Current,
    Final,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Register16 {
    AX,
    CX,
    DX,
    BX,
    SP,
    BP,
    SI,
    DI,
    ES,
    FS,
    GS,
    CS,
    SS,
    DS,
    PC,
    InvalidRegister,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Register32 {
    EAX,
    ECX,
    EDX,
    EBX,
    ESP,
    EBP,
    ESI,
    EDI,
    EIP,
    InvalidRegister,
}

pub enum FileOpenContext {
    LoadWorkspace,
    LoadProgramSource,
    LoadProgramBinary,
    LoadRegisterBinary,
}

pub enum FileSaveContext {
    SaveWorkspace,
    SaveProgramSource,
    SaveProgramBinary,
    SaveRegisterBinary,
}

#[derive(Clone, serde::Serialize, serde::Deserialize, Debug)]
pub enum BinaryBlobType {
    Program,
    Data,
    Registers,
}

#[derive(Copy, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq, Debug)]
pub enum MountAddress {
    CsIp,
    FlatAddress(u32),
}

impl MountAddress {
    pub fn flat_address(&self) -> Option<u32> {
        match self {
            MountAddress::CsIp => None,
            MountAddress::FlatAddress(addr) => Some(*addr),
        }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum ClientControlState {
    #[default]
    Setup,
    Running,
    Error,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ScheduleType {
    OneShot,
    Repeat,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ClientExecuteState {
    SingleCycleStep,
    SingleInstructionStep,
    Run,
}
