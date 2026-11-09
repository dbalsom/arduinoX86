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

//! Client-side decoding of a bounded memory snapshot. This never clocks the CPU.
use anyhow::Context;
use arduinox86_client::{CpuClient, ServerCpuType};
use marty_dasm::prelude::{format_instruction, CpuType, Decoder, DecoderOptions, FormatOptions, SegmentSize};

pub const MAX_DISASSEMBLY_BYTES: u32 = 0x1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DisassemblyRequest {
    pub id: u64,
    pub address: u32,
    pub size: u32,
    pub ip: u32,
    pub cs: Option<u16>,
    pub bits32: bool,
}

#[derive(Clone, Debug)]
pub struct DisassemblyRow {
    pub address: u32,
    pub ip: u32,
    pub bytes: String,
    pub instruction: String,
}

pub struct DisassemblyReadout {
    pub request: DisassemblyRequest,
    pub result:  Result<Vec<DisassemblyRow>, String>,
}

fn decoder_options(cpu: ServerCpuType, bits32: bool) -> Result<DecoderOptions, String> {
    let model = match cpu {
        ServerCpuType::Intel8088 | ServerCpuType::Intel8086 => CpuType::Intel808x,
        ServerCpuType::NecV20 | ServerCpuType::NecV30 => CpuType::NecVx0,
        ServerCpuType::Intel80286 => CpuType::Intel80286,
        ServerCpuType::Intel80386 => CpuType::Intel80386,
        ServerCpuType::Intel80188(_) | ServerCpuType::Intel80186(_) => {
            // The upstream dispatcher still uses unimplemented! for this model.
            return Err("marty_dasm does not implement 80186/80188 decoding yet.".into());
        }
        ServerCpuType::Undetected => return Err("CPU model is not detected.".into()),
    };
    if bits32 && model != CpuType::Intel80386 {
        return Err("32-bit code segments require a 386.".into());
    }
    Ok(DecoderOptions {
        cpu: model,
        segment_size: if bits32 {
            SegmentSize::Segment32
        }
        else {
            SegmentSize::Segment16
        },
        ..Default::default()
    })
}

pub fn validate_request(cpu: ServerCpuType, request: DisassemblyRequest) -> Result<(), String> {
    decoder_options(cpu, request.bits32)?;
    let end = u64::from(request.address) + u64::from(request.size);
    if request.size == 0 || request.size > MAX_DISASSEMBLY_BYTES || end > (1u64 << cpu.address_bus_size()) {
        return Err("Address range is outside the CPU bus or size is not 1..1000 hex bytes.".into());
    }
    Ok(())
}

pub fn read_disassembly(
    client: &mut CpuClient,
    cpu: ServerCpuType,
    request: DisassemblyRequest,
) -> anyhow::Result<DisassemblyReadout> {
    validate_request(cpu, request).map_err(anyhow::Error::msg)?;
    let mut bytes = Vec::with_capacity(request.size as usize);
    client
        .read_memory(request.address, request.size, &mut bytes)
        .with_context(|| format!("Reading {} disassembly bytes at {:08X}h", request.size, request.address))?;
    Ok(DisassemblyReadout {
        request,
        result: decode(cpu, request, &bytes),
    })
}

pub fn decode(cpu: ServerCpuType, request: DisassemblyRequest, bytes: &[u8]) -> Result<Vec<DisassemblyRow>, String> {
    validate_request(cpu, request)?;
    if bytes.len() != request.size as usize {
        return Err("Incomplete disassembly memory read.".into());
    }
    let options = decoder_options(cpu, request.bits32)?;
    let mut rows = Vec::new();
    let mut offset = 0;
    while offset < bytes.len() {
        let ip = request.ip.wrapping_add(offset as u32);
        let ip = if request.bits32 { ip } else { ip & 0xffff };
        let mut decoder = Decoder::new(&bytes[offset..], options);
        let decoded = decoder.decode_next();
        let consumed = decoder.position() as usize;
        let (size, instruction) = match decoded {
            Ok(instruction) if consumed > 0 && consumed <= bytes.len() - offset && instruction.is_complete => (
                consumed,
                format_instruction(
                    &instruction,
                    &FormatOptions {
                        ip,
                        ..Default::default()
                    },
                ),
            ),
            _ => (1, format!("db {:02X}h ; undecodable or incomplete", bytes[offset])),
        };
        rows.push(DisassemblyRow {
            address: request.address + offset as u32,
            ip,
            bytes: bytes[offset..offset + size]
                .iter()
                .map(|b| format!("{b:02X}"))
                .collect::<Vec<_>>()
                .join(" "),
            instruction,
        });
        offset += size;
    }
    Ok(rows)
}

/// Locate the final instruction in a complete 16-bit program. Unlike the
/// memory viewer, endpoint selection must not recover errors as single-byte DBs.
/// Decode lengths only, without formatting rows or imposing the viewer's limit.
pub fn last_instruction_offset(cpu: ServerCpuType, bytes: &[u8]) -> Result<usize, String> {
    if bytes.is_empty() {
        return Err("Program contains no instructions.".into());
    }
    let mut decoder = Decoder::new(bytes, decoder_options(cpu, false)?);
    let mut offset = 0;
    let mut last = 0;
    while offset < bytes.len() {
        let instruction = decoder.decode_next()
            .map_err(|error| format!("Cannot decode instruction at program offset {offset:X}h: {error}"))?;
        let next = decoder.position() as usize;
        // CPU-invalid opcodes can still have a decoded length and are useful
        // in hardware fault tests. Require completeness, not instruction validity.
        if !instruction.is_complete || next <= offset || next > bytes.len() {
            return Err(format!("Incomplete instruction at program offset {offset:X}h."));
        }
        last = offset;
        offset = next;
    }
    Ok(last)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(size: u32) -> DisassemblyRequest {
        DisassemblyRequest {
            id: 1,
            address: 0x12103,
            size,
            ip: 0x100,
            cs: Some(0x18),
            bits32: false,
        }
    }

    #[test]
    fn decodes_286_system_ops_and_branches() {
        let rows = decode(
            ServerCpuType::Intel80286,
            request(7),
            &[0x0f, 0x05, 0xb8, 0x34, 0x12, 0xeb, 0xf9],
        )
        .unwrap();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0].instruction, "loadall");
        assert_eq!(rows[1].bytes, "B8 34 12");
        assert_eq!(rows[2].address, 0x12108);
        assert_eq!(rows[2].ip, 0x105);
        assert!(rows[2].instruction.contains("100h"), "{}", rows[2].instruction);
        let older = decode(ServerCpuType::Intel8088, request(2), &[0x0f, 0x05]).unwrap();
        assert_ne!(older[0].instruction, rows[0].instruction);
    }

    #[test]
    fn handles_truncation_and_unknown_cpus() {
        let rows = decode(ServerCpuType::Intel80286, request(2), &[0xb8, 0x34]).unwrap();
        assert_eq!(rows.len(), 2);
        assert!(rows[0].instruction.starts_with("db B8h"));
        assert!(decode(ServerCpuType::Intel80186(false), request(1), &[0x90]).is_err());
        let mut invalid = request(2);
        invalid.address = 0xffffff;
        assert!(validate_request(ServerCpuType::Intel80286, invalid).is_err());
        invalid = request(MAX_DISASSEMBLY_BYTES + 1);
        assert!(validate_request(ServerCpuType::Intel80286, invalid).is_err());
    }

    #[test]
    fn endpoint_includes_prefix_and_operand() {
        for bytes in [
            &[0x40, 0xB8, 0x34, 0x90][..], // Final 90h is an immediate byte.
            &[0x40, 0xF3, 0x26, 0xA4][..], // REP ES: MOVSB
            &[0x40, 0xC7, 0x86, 0x34, 0x12, 0x78, 0x56][..], // MOV [BP+1234h],5678h
            &[0x40, 0x0F, 0x01, 0x16, 0x00, 0x30][..], // LGDT [3000h]
        ] {
            assert_eq!(last_instruction_offset(ServerCpuType::Intel80286, bytes).unwrap(), 1);
        }
        // 0F 05 is LOADALL on the 286, but POP CS / ADD AX,9090h on the 8088.
        let bytes = [0x0F, 0x05, 0x90, 0x90];
        assert_eq!(last_instruction_offset(ServerCpuType::Intel80286, &bytes).unwrap(), 3);
        assert_eq!(last_instruction_offset(ServerCpuType::Intel8088, &bytes).unwrap(), 1);
        let full_segment = vec![0x90; 0x10000];
        assert_eq!(last_instruction_offset(ServerCpuType::Intel80286, &full_segment).unwrap(), 0xFFFF);
    }

    #[test]
    fn endpoint_requires_complete_decode() {
        for bytes in [&[][..], &[0x90, 0xB8, 0x34][..], &[0x90, 0xF3][..]] {
            assert!(last_instruction_offset(ServerCpuType::Intel80286, bytes).is_err(), "{bytes:02X?}");
        }
        assert_eq!(last_instruction_offset(ServerCpuType::Intel80286, &[0x0F, 0xFF, 0x90]).unwrap(), 2);
        assert!(last_instruction_offset(ServerCpuType::Intel80186(false), &[0x90]).is_err());
    }

    #[test]
    #[ignore = "requires ARDUINOX86_PORT and exclusive access to the GIGA; replaces emulated memory"]
    fn hardware_bulk_reads_both_backends() -> anyhow::Result<()> {
        use anyhow::ensure;
        use arduinox86_client::{MemoryStrategy, ServerFlags};
        let mut client = CpuClient::init(Some(std::env::var("ARDUINOX86_PORT")?), Some(5000))?;
        let result = (|| -> anyhow::Result<()> {
            for backend in [0, ServerFlags::USE_SDRAM_BACKEND] {
                client.reset_cpu()?;
                let flags = ServerFlags::ENABLE_CYCLE_LOGGING | backend;
                client.set_flags(flags)?;
                client.erase_memory()?;
                let pattern: Vec<u8> = (0..4102).map(|i| ((i * 73) ^ (i >> 3)) as u8).collect();
                client.set_memory(0x10001, &pattern)?;
                let cycle = client.server_status()?.cycle_ct;
                for offset in [0, 1] {
                    for size in [0, 1, 2, 3, 4, 5, 255, 256, 257, 1023, 1024, 1025, 4096] {
                        let mut bytes = Vec::new();
                        client.read_memory(0x10001 + offset, size, &mut bytes)?;
                        ensure!(
                            bytes == pattern[offset as usize..(offset + size) as usize],
                            "bulk read mismatch: backend {backend:#x}, offset {offset}, size {size}"
                        );
                    }
                }
                if backend == 0 {
                    // Sparse defaults and high addresses must agree with scalar reads.
                    for strategy in [MemoryStrategy::Zero, MemoryStrategy::Ones, MemoryStrategy::Random] {
                        client.set_memory_strategy(strategy, 0, 0x1000000)?;
                        let mut scalar = Vec::new();
                        for address in 0xf10001..0xf10008 {
                            client.read_memory(address, 1, &mut scalar)?;
                        }
                        let mut bulk = Vec::new();
                        client.read_memory(0xf10001, 7, &mut bulk)?;
                        ensure!(bulk == scalar, "generated default mismatch");
                    }
                    client.set_memory(0xff0001, &pattern)?;
                    let mut high = Vec::new();
                    client.read_memory(0xff0001, pattern.len() as u32, &mut high)?;
                    ensure!(high == pattern, "sparse high-address read mismatch");
                }
                else {
                    ensure!(
                        client.read_memory(0xffffff, 3, &mut Vec::new()).is_err(),
                        "SDRAM bounds accepted"
                    );
                }
                ensure!(
                    client.read_memory(u32::MAX, 2, &mut Vec::new()).is_err(),
                    "wrapped range accepted"
                );
                ensure!(client.get_flags()? == flags, "protocol desynchronized after rejection");
                ensure!(client.server_status()?.cycle_ct == cycle, "memory read clocked CPU");
                println!(
                    "PASS {}: bulk boundaries, byte alignment, defaults and protocol framing",
                    if backend == 0 { "hash" } else { "SDRAM" }
                );
            }
            Ok(())
        })();
        let reset = client.reset_cpu();
        let flags = client.set_flags(ServerFlags::ENABLE_CYCLE_LOGGING);
        result?;
        reset?;
        flags?;
        Ok(())
    }
}
