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

//! 286 trap execution. The saved CS:IP is the next execution point, not the
//! address of the instruction which raised the trap. Queries never clock the CPU.
use crate::{CpuClient, CpuClientError, RemoteCpuRegistersV2, ServerCapabilities, ServerCommand, ServerFlags};
use binrw::{BinRead, BinReaderExt};
use std::{io::Cursor, time::Duration};

pub const TRAP_FLAG: u16 = 0x0100;
pub const MANAGED_IDT_BASE: u32 = 0x003000;
pub const MANAGED_IDT_LIMIT_REAL: u16 = 0x03FF;
pub const MANAGED_IDT_LIMIT_PROTECTED: u16 = 0x07FF;
pub const MANAGED_HANDLER_BASE: u32 = 0x005000;
pub const DEFAULT_REAL_MODE_TRAP_HANDLER: u32 = MANAGED_HANDLER_BASE + 0x10;

/// Defaults for the GUI's managed environment. Raw LOADALL callers can still
/// provide custom tables; protected-mode gates and descriptors live in memory.
pub fn configure_managed_trap_registers(regs: &mut RemoteCpuRegistersV2) {
    regs.flags |= TRAP_FLAG;
    regs.idt_desc.set_base_address(MANAGED_IDT_BASE);
    regs.idt_desc.set_limit(if regs.msw & 1 != 0 {
        MANAGED_IDT_LIMIT_PROTECTED
    } else {
        MANAGED_IDT_LIMIT_REAL
    });
}

#[derive(BinRead, Copy, Clone, Debug, PartialEq, Eq)]
#[br(repr = u8)]
pub enum TrapStatus {
    Idle,
    Ready,
    Running,
    Paused,
    Captured,
    Failed,
}

#[derive(BinRead, Copy, Clone, Debug, PartialEq, Eq)]
#[br(repr = u8)]
pub enum TrapReason {
    None,
    Step,
    Endpoint,
    Exception,
    Timeout,
    Shutdown,
    Halt,
    InvalidFrame,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct InstructionAddress {
    pub cs: u16,
    pub ip: u16,
}

/// Last observed guest boundary. While running, this is not a live register view.
/// `start_*` comes from LOADALL or the previous resume address; REP and interrupt
/// shadows can make one observed interval cover a partial/multiple instruction.
#[derive(BinRead, Clone, Debug, PartialEq, Eq)]
#[br(little)]
pub struct InstructionState {
    #[br(assert(version == 1))]
    pub version: u8,
    pub status: TrapStatus,
    pub reason: TrapReason,
    pub vector: u8,
    pub run_id: u32,
    pub sequence: u32,
    pub start_cs: u16,
    pub start_ip: u16,
    pub resume_cs: u16,
    pub resume_ip: u16,
    pub flags: u16,
    /// Loaded SS:SP before execution, or saved old SS:SP from a stack switch.
    /// Zero and unavailable for a same-ring trap; check STACK_VALID.
    pub ss: u16,
    pub sp: u16,
    pub frame_bytes: u8,
    pub validity: u8,
    pub frame_address: u32,
    pub trap_cycle: u32,
    pub resume_cycle: u32,
    pub handler_address: u32,
    pub capture_cycle: u32,
}

impl InstructionState {
    pub const FRAME_VALID: u8 = 0x01;
    pub const STACK_VALID: u8 = 0x02;
    pub const PROTECTED_MODE: u8 = 0x04;
    pub const STACK_SWITCHED: u8 = 0x08;

    pub const WIRE_SIZE: usize = 48;
    pub fn resume_address(&self) -> InstructionAddress {
        InstructionAddress {
            cs: self.resume_cs,
            ip: self.resume_ip,
        }
    }
    pub fn has_frame(&self) -> bool {
        self.validity & InstructionState::FRAME_VALID != 0
    }
    pub fn stack_switched(&self) -> bool {
        self.validity & InstructionState::STACK_SWITCHED != 0
    }
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CpuClientError> {
        if bytes.len() != Self::WIRE_SIZE {
            return Err(CpuClientError::TypeConversionError);
        }
        Cursor::new(bytes)
            .read_le()
            .map_err(|_| CpuClientError::TypeConversionError)
    }
}

/// Preserve the actual STOREALL image, plus the trap's saved guest frame.
/// Hidden caches which entry overwrote cannot be reconstructed from selectors.
#[derive(Clone, Debug)]
pub struct TrapCapture286 {
    pub instruction: InstructionState,
    /// Unmodified handler STOREALL image. At a zero-instruction endpoint this
    /// is the initial LOADALL image instead
    pub handler_registers: RemoteCpuRegistersV2,
}

impl TrapCapture286 {
    pub const WIRE_SIZE: usize = InstructionState::WIRE_SIZE + 102;

    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CpuClientError> {
        if bytes.len() != Self::WIRE_SIZE {
            return Err(CpuClientError::TypeConversionError);
        }
        Ok(Self {
            instruction: InstructionState::from_bytes(&bytes[..InstructionState::WIRE_SIZE])?,
            handler_registers: RemoteCpuRegistersV2::try_from(&bytes[InstructionState::WIRE_SIZE..])
                .map_err(|_| CpuClientError::TypeConversionError)?,
        })
    }
}

impl CpuClient {
    fn trap_reply(&mut self, command: ServerCommand, len: usize) -> Result<Vec<u8>, CpuClientError> {
        self.send_command_byte(command)?;
        self.read_result_code(command)?;
        let mut bytes = vec![0; len];
        self.recv_buf(&mut bytes)?;
        self.read_result_code(command)?;
        Ok(bytes)
    }

    pub fn enable_trap_execution(&mut self) -> Result<(), CpuClientError> {
        if self.capabilities() & ServerCapabilities::TRAP_EXECUTION_286 == 0 {
            return Err(CpuClientError::BadParameter(
                "Server does not support 286 trap execution".into(),
            ));
        }
        let flags = self.get_flags()?;
        self.set_flags(flags | ServerFlags::ENABLE_TRAP_EXECUTION)?;
        Ok(())
    }

    pub fn set_trap_endpoint(&mut self, endpoint: Option<InstructionAddress>) -> Result<(), CpuClientError> {
        let mut bytes = [0; 5];
        if let Some(address) = endpoint {
            bytes[0] = 1;
            bytes[1..3].copy_from_slice(&address.cs.to_le_bytes());
            bytes[3..5].copy_from_slice(&address.ip.to_le_bytes());
        }
        self.send_command_byte(ServerCommand::CmdSetTrapEndpoint)?;
        self.send_buf(&bytes)?;
        self.read_result_code(ServerCommand::CmdSetTrapEndpoint)?;
        Ok(())
    }

    /// Select the real-mode handler entry for subsequent trap loads. Survives
    /// CPU reset; protected mode uses the address resolved from the supplied gate.
    pub fn set_real_mode_trap_handler(&mut self, address: u32) -> Result<(), CpuClientError> {
        if self.capabilities() & ServerCapabilities::SERVER_TRAP_HANDLER_286 == 0 {
            return Err(CpuClientError::BadParameter(
                "Server does not support installed 286 trap handlers".into(),
            ));
        }
        self.send_command_byte(ServerCommand::CmdSetRealModeTrapHandler)?;
        self.send_buf(&address.to_le_bytes())?;
        self.read_result_code(ServerCommand::CmdSetRealModeTrapHandler)?;
        Ok(())
    }

    pub fn instruction_state(&mut self) -> Result<InstructionState, CpuClientError> {
        InstructionState::from_bytes(&self.trap_reply(ServerCommand::CmdGetInstructionState, 48)?)
    }

    fn run_trap_command(&mut self, command: ServerCommand, cycles: u32) -> Result<InstructionState, CpuClientError> {
        if !(1..=1_000_000).contains(&cycles) {
            return Err(CpuClientError::BadParameter(
                "Trap budget must be 1..1000000 cycles".into(),
            ));
        }
        let previous = self.port.borrow().timeout();
        self.port
            .borrow_mut()
            .set_timeout(previous.max(Duration::from_secs(30)))
            .map_err(CpuClientError::ConfigurePort)?;
        let result = (|| {
            self.send_command_byte(command)?;
            self.send_buf(&cycles.to_le_bytes())?;
            self.read_result_code(command)?;
            self.instruction_state()
        })();
        let restored = self
            .port
            .borrow_mut()
            .set_timeout(previous)
            .map_err(CpuClientError::ConfigurePort);
        let state = result?;
        restored?;
        Ok(state)
    }

    pub fn step_instruction(&mut self, cycle_budget: u32) -> Result<InstructionState, CpuClientError> {
        self.run_trap_command(ServerCommand::CmdStepInstruction, cycle_budget)
    }

    pub fn continue_trap(&mut self, cycle_budget: u32) -> Result<InstructionState, CpuClientError> {
        self.run_trap_command(ServerCommand::CmdContinueTrap, cycle_budget)
    }

    /// Read a completed instruction-step or endpoint capture. Never advances the CPU.
    pub fn trap_capture(&mut self) -> Result<TrapCapture286, CpuClientError> {
        TrapCapture286::from_bytes(&self.trap_reply(ServerCommand::CmdGetTrapCapture, TrapCapture286::WIRE_SIZE)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SegmentDescriptorV1;

    // Physical MOV AX,1234 / INC AX observation:
    // same-ring SS:SP is unavailable, and there are no cache provenance bits.
    fn wire() -> Vec<u8> {
        let hex = "0104020101000000020000001800030118000401060100000000060503a002008e01000051010000076001004d020000";
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn validates_boundary_wire_records() {
        let mut b = wire();
        let s = InstructionState::from_bytes(&b).unwrap();
        assert_eq!(s.status, TrapStatus::Captured);
        assert_eq!((s.start_ip, s.resume_ip), (0x103, 0x104));
        assert_eq!((s.frame_address, s.trap_cycle, s.capture_cycle), (0x2a003, 398, 589));
        assert!(s.has_frame());
        assert_eq!(s.validity & InstructionState::STACK_VALID, 0);
        assert!(InstructionState::from_bytes(&b[..47]).is_err());
        b[0] = 0xff;
        assert!(InstructionState::from_bytes(&b).is_err());
        b[0] = 1;
        b[1] = 0xff;
        assert!(InstructionState::from_bytes(&b).is_err());
    }

    #[test]
    fn separates_handler_and_guest_capture() {
        let regs = RemoteCpuRegistersV2 {
            ax: 0x1235,
            cs: 8,
            ip: 4,
            flags: 6,
            ss: 0x10,
            sp: 0x7ff6,
            cs_desc: SegmentDescriptorV1::new().with_base_address(0x16007),
            ss_desc: SegmentDescriptorV1::new().with_base_address(0x20005),
            ..Default::default()
        };
        let mut b = wire();
        b[16..18].copy_from_slice(&0x1bu16.to_le_bytes());
        b[22..24].copy_from_slice(&0x23u16.to_le_bytes());
        b[24..26].copy_from_slice(&0x4567u16.to_le_bytes());
        b[26] = 10;
        b[27] |= InstructionState::STACK_VALID | InstructionState::STACK_SWITCHED;
        let mut raw = Vec::new();
        regs.to_buffer(&mut raw);
        b.extend_from_slice(&raw);
        let c = TrapCapture286::from_bytes(&b).unwrap();
        assert_eq!(
            (c.instruction.resume_cs, c.instruction.ss, c.instruction.sp),
            (0x1b, 0x23, 0x4567)
        );
        let mut roundtrip = Vec::new();
        c.handler_registers.to_buffer(&mut roundtrip);
        assert_eq!(roundtrip, raw);
        assert_eq!(
            (c.handler_registers.cs, c.handler_registers.ss, c.handler_registers.sp),
            (8, 0x10, 0x7ff6)
        );
        assert!(TrapCapture286::from_bytes(&b[..b.len() - 1]).is_err());
        b.push(0); // Trailing data is not part of a capture.
        assert!(TrapCapture286::from_bytes(&b).is_err());
    }
}
