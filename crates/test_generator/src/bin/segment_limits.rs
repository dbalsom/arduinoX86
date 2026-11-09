//! Deterministic segment boundary captures, using the existing CPU server.
use std::{
    collections::BTreeSet,
    fs::{self, File},
    io::{BufWriter, Cursor, Write},
    path::{Path, PathBuf},
    process::Command,
    time::{Duration, Instant},
};

use anyhow::{ensure, Context, Result};
use arduinox86_client::{
    BinWrite, BusState, CpuClient, ProgramState, RegisterSetType, Registers32, RemoteCpuRegistersV2,
    RemoteCpuRegistersV3A, ServerCpuType, ServerCycleState, ServerFlags,
};
use clap::{Parser, ValueEnum};
use serde::Serialize;

const CODE_BASE: u32 = 0x10000;
const DATA_BASE: u32 = 0x20000;
const RESULT_SIZE: usize = 28;
const SOURCE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../asm/tests/segment_limits.asm");

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, ValueEnum)]
enum Cpu {
    #[value(name = "286")]
    I286,
    #[value(name = "386")]
    I386,
}

impl Cpu {
    fn number(self) -> u32 {
        match self {
            Self::I286 => 286,
            Self::I386 => 386,
        }
    }

    fn server(self) -> ServerCpuType {
        match self {
            Self::I286 => ServerCpuType::Intel80286,
            Self::I386 => ServerCpuType::Intel80386,
        }
    }
}

#[derive(Parser)]
#[command(about = "Capture segment limit behavior on a 286 or 386EX with ArduinoX86")]
struct Args {
    #[arg(long, value_enum)]
    cpu: Cpu,
    /// List case names without connecting to hardware or assembling anything.
    #[arg(long, conflicts_with = "assemble_only")]
    list: bool,
    /// Assemble fixtures and LOADALL state without connecting to hardware.
    #[arg(long)]
    assemble_only: bool,
    /// Run only case names containing this text.
    #[arg(long)]
    filter: Option<String>,
    /// A new directory for fixtures, raw observations and cycle traces.
    #[arg(long, required_unless_present = "list")]
    output: Option<PathBuf>,
    #[arg(long)]
    com_port: Option<String>,
    /// Physical chip markings and revision, recorded verbatim with observations.
    #[arg(long)]
    chip: Option<String>,
    /// Board, shield and firmware revision used for capture.
    #[arg(long)]
    board: Option<String>,
    #[arg(long, default_value = "nasm")]
    nasm: PathBuf,
    #[arg(long, default_value_t = 2, value_parser = clap::value_parser!(u32).range(1..))]
    repeat: u32,
    #[arg(long, default_value_t = 20000, value_parser = clap::value_parser!(u32).range(1..))]
    max_cycles: u32,
    #[arg(long, default_value_t = 120, value_parser = clap::value_parser!(u64).range(1..))]
    timeout_seconds: u64,
}

#[derive(Clone, Debug, Serialize)]
struct Case {
    name: String,
    cpu: Cpu,
    protected: bool,
    down: bool,
    big: bool,
    limit: u32,
    offset: u32,
    address32: bool,
    stack: bool,
    width: u8,
    write: bool,
    direct: bool,
}

fn cases(cpu: Cpu) -> Vec<Case> {
    let mut cases = Vec::new();
    let mut descriptors = vec![(false, false, false, 0xffff_u32)];
    for down in [false, true] {
        for limit in [0, 1, 0xfffe, 0xffff] {
            descriptors.push((true, down, false, limit));
            if cpu == Cpu::I386 {
                descriptors.push((true, down, true, limit));
            }
        }
    }
    if cpu == Cpu::I386 {
        descriptors.extend([
            (true, false, true, 0xffffefff),
            (true, false, true, 0xffffffff),
            (true, true, true, 0xffffffff),
        ]);
    }

    for (protected, down, big, limit) in descriptors {
        for width in [1, 2, 4] {
            if cpu == Cpu::I286 && width == 4 {
                continue;
            }
            let upper = if big { u32::MAX } else { 0xffff };
            let offsets = BTreeSet::from([
                0,
                limit.saturating_sub(width as u32 - 1),
                limit,
                limit.saturating_add(1),
                upper - width as u32 + 1,
                upper,
            ]);
            for offset in offsets {
                for address32 in [false, true] {
                    if (address32 && cpu == Cpu::I286) || (!address32 && offset > 0xffff) {
                        continue;
                    }
                    for stack in [false, true] {
                        // The handler's frame at 7ff4..7fff must remain valid.
                        // Empty/tiny segments are still tested through DS.
                        if stack && protected && ((down && limit >= 0x7ff4) || (!down && limit < 0x8000)) {
                            continue;
                        }
                        for write in [false, true] {
                            for direct in [false, true] {
                                let mode = if protected { "pm" } else { "rm" };
                                let direction = if down { "down" } else { "up" };
                                let segment = if stack { "ss" } else { "ds" };
                                let access = if write { "write" } else { "read" };
                                let encoding = if direct { "moffs" } else { "modrm" };
                                let name = format!(
                                    "{mode}-{direction}-b{}-l{limit:08x}-o{offset:08x}-a{}-{segment}-{encoding}-{access}{}",
                                    u8::from(big), if address32 { 32 } else { 16 }, width * 8
                                );
                                cases.push(Case {
                                    name,
                                    cpu,
                                    protected,
                                    down,
                                    big,
                                    limit,
                                    offset,
                                    address32,
                                    stack,
                                    width,
                                    write,
                                    direct,
                                });
                            }
                        }
                    }
                }
            }
        }
    }
    cases
}

#[derive(Debug)]
struct Fixture {
    bytes: Vec<u8>,
    entry: u16,
    probe: u16,
    result: u16,
    stopped: u16,
}

fn word(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes(bytes[at..at + 2].try_into().unwrap())
}

impl Fixture {
    fn parse(bytes: Vec<u8>) -> Result<Self> {
        ensure!(
            bytes.len() >= 18 && &bytes[..6] == b"SLIM\x01\x00",
            "invalid fixture header"
        );
        ensure!(usize::from(word(&bytes, 16)) == bytes.len(), "invalid fixture length");
        for at in [6, 8, 10, 12, 14] {
            ensure!(usize::from(word(&bytes, at)) < bytes.len(), "invalid fixture offset");
        }
        ensure!(
            usize::from(word(&bytes, 12)) + RESULT_SIZE == bytes.len(),
            "invalid result buffer"
        );
        Ok(Self {
            entry: word(&bytes, 6),
            probe: word(&bytes, 8),
            result: word(&bytes, 12),
            stopped: word(&bytes, 14),
            bytes,
        })
    }
}

fn assemble(case: &Case, nasm: &Path, path: &Path) -> Result<Fixture> {
    let mut command = Command::new(nasm);
    command.args(["-f", "bin", "-Werror", SOURCE, "-o"]).arg(path);
    configure_assembler(case, &mut command);
    let output = command.output().context("starting NASM")?;
    ensure!(
        output.status.success(),
        "NASM failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    Fixture::parse(fs::read(path)?)
}

fn configure_assembler(case: &Case, command: &mut Command) {
    for (key, value) in [
        ("CPU", case.cpu.number()),
        ("PROTECTED", u32::from(case.protected)),
        ("DOWN", u32::from(case.down)),
        ("BIG", u32::from(case.big)),
        ("LIMIT", case.limit),
        ("OFFSET", case.offset),
        ("ADDRESS32", u32::from(case.address32)),
        ("STACK", u32::from(case.stack)),
        ("WIDTH", u32::from(case.width)),
        ("WRITE", u32::from(case.write)),
        ("DIRECT", u32::from(case.direct)),
    ] {
        command.arg(format!("-D{key}={value}"));
    }
}

fn initial_registers(cpu: Cpu, entry: u16) -> Result<Vec<u8>> {
    let mut buffer = Cursor::new(Vec::new());
    match cpu {
        Cpu::I286 => {
            let mut regs = RemoteCpuRegistersV2 {
                cs: 0x1000,
                ip: entry,
                ss: 0x3000,
                sp: 0x8000,
                ..Default::default()
            };
            regs.normalize_descriptors();
            regs.to_buffer(&mut buffer);
        }
        Cpu::I386 => {
            let mut regs = RemoteCpuRegistersV3A {
                cs: 0x1000,
                eip: u32::from(entry),
                ss: 0x3000,
                esp: 0x8000,
                eflags: 2,
                ..Default::default()
            };
            regs.normalize_descriptors();
            regs.write_le(&mut buffer)?;
        }
    }
    Ok(buffer.into_inner())
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct Memory {
    address: u32,
    bytes: Vec<u8>,
}

fn input_memory(case: &Case) -> Vec<Memory> {
    // All these physical addresses are below 1 MiB, including the 4 GiB edge
    // after adding DATA_BASE. No result depends on uncaptured upper address pins.
    let address = DATA_BASE.wrapping_add(case.offset);
    let mut addresses = BTreeSet::new();
    for start in [address, DATA_BASE, DATA_BASE + (case.offset & 0xffff)] {
        for byte in start - 4..start + 8 {
            addresses.insert(byte);
        }
    }
    addresses
        .into_iter()
        .map(|address| Memory {
            address,
            bytes: vec![(address as u8).wrapping_mul(37).wrapping_add((address >> 16) as u8)],
        })
        .collect()
}

#[derive(Debug, PartialEq, Eq, Serialize)]
struct Outcome {
    complete: bool,
    vector: u16,
    error_code: u16,
    saved_ip: u16,
    saved_cs: u16,
    saved_flags: u16,
    value: u32,
    stack_before: u32,
    stack_after: u32,
    msw: u16,
}

fn outcome(bytes: &[u8]) -> Result<Outcome> {
    ensure!(bytes.len() == RESULT_SIZE, "short result buffer");
    let dword = |at| u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
    Ok(Outcome {
        complete: word(bytes, 0) == 0x534c,
        vector: word(bytes, 2),
        error_code: word(bytes, 4),
        saved_ip: word(bytes, 6),
        saved_cs: word(bytes, 8),
        saved_flags: word(bytes, 10),
        value: dword(12),
        stack_before: dword(16),
        stack_after: dword(20),
        msw: word(bytes, 24),
    })
}

#[derive(Serialize)]
struct Observation<'a> {
    format_version: u32,
    chip: &'a str,
    board: &'a str,
    case: &'a Case,
    attempt: u32,
    termination: String,
    clocks: u32,
    load_handoff_cycles: usize,
    logged_cycles: usize,
    probe_ip: u16,
    initial_memory: Vec<Memory>,
    final_memory: Vec<Memory>,
    result: Outcome,
}

fn save_cycles(path: &Path, cycles: &[ServerCycleState]) -> Result<()> {
    let mut file = BufWriter::new(File::create(path)?);
    writeln!(
        file,
        "cycle,address_bus,data_bus,cpu_state_bits,cpu_status_bits,bus_control_bits,bus_command_bits,pins"
    )?;
    for (i, cycle) in cycles.iter().enumerate() {
        writeln!(
            file,
            "{i},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x}",
            cycle.address_bus,
            cycle.data_bus,
            cycle.cpu_state_bits,
            cycle.cpu_status_bits,
            cycle.bus_control_bits,
            cycle.bus_command_bits,
            cycle.pins
        )?;
    }
    file.flush()?;
    Ok(())
}

fn capture(args: &Args, client: &mut CpuClient, case: &Case, fixture: &Fixture, regs: &[u8], dir: &Path) -> Result<()> {
    let mut previous = None;
    for attempt in 1..=args.repeat {
        client.erase_memory()?;
        client.set_memory(CODE_BASE, &fixture.bytes)?;
        let memory = input_memory(case);
        for region in &memory {
            client.set_memory(region.address, &region.bytes)?;
        }
        client.set_program_bounds(CODE_BASE, CODE_BASE + fixture.bytes.len() as u32)?;
        client.clear_cycle_log()?;
        client.load_registers_from_buf(RegisterSetType::from(case.cpu.server()), regs)?;
        // LOADALL has finished. Stepping avoids automatic real mode IVT handling
        // and HLT/NMI finalization, neither of which describes this PM program.
        let mut cycles = client.get_cycle_states()?;
        let load_handoff_cycles = cycles.len();
        client.clear_cycle_log()?;
        let start = Instant::now();
        let mut clocks = 0;
        let termination = loop {
            if clocks >= args.max_cycles || start.elapsed() >= Duration::from_secs(args.timeout_seconds) {
                break "timeout".to_owned();
            }
            let cycle = client.get_cycle_state(true)?;
            clocks += 1;
            // Drain before even the smallest server ring buffer can wrap.
            if clocks % 256 == 0 {
                cycles.extend(client.get_cycle_states()?);
                client.clear_cycle_log()?;
            }
            if cycle.program_state != ProgramState::Execute {
                break format!("server_{:?}", cycle.program_state);
            }
            if cycle.ale()
                && case.cpu.server().decode_status(cycle.cpu_status_bits) == BusState::CODE
                && cycle.address_bus == CODE_BASE + u32::from(fixture.stopped)
            {
                break "rendezvous".to_owned();
            }
        };
        if clocks % 256 != 0 {
            cycles.extend(client.get_cycle_states()?);
        }
        save_cycles(&dir.join(format!("{attempt}.cycles.csv")), &cycles)?;
        let mut raw = Vec::new();
        client.read_memory(CODE_BASE + u32::from(fixture.result), RESULT_SIZE as u32, &mut raw)?;
        fs::write(dir.join(format!("{attempt}.result.bin")), &raw)?;
        let result = outcome(&raw)?;
        let mut final_memory = Vec::new();
        for region in &memory {
            let mut bytes = Vec::new();
            client.read_memory(region.address, region.bytes.len() as u32, &mut bytes)?;
            final_memory.push(Memory {
                address: region.address,
                bytes,
            });
        }
        let observation = Observation {
            format_version: 1,
            chip: args.chip.as_deref().unwrap(),
            board: args.board.as_deref().unwrap(),
            case,
            attempt,
            termination,
            clocks,
            load_handoff_cycles,
            logged_cycles: cycles.len(),
            probe_ip: fixture.probe,
            initial_memory: memory,
            final_memory,
            result,
        };
        fs::write(
            dir.join(format!("{attempt}.toml")),
            toml::to_string_pretty(&observation)?,
        )?;
        ensure!(
            observation.termination == "rendezvous" && observation.result.complete,
            "{} did not complete; partial observation saved",
            case.name
        );
        ensure!(
            observation.result.vector == 0xffff
                || (observation.result.saved_ip == fixture.probe
                    && observation.result.saved_cs == if case.protected { 8 } else { 0x1000 }),
            "{} faulted outside the probe; observation saved",
            case.name
        );
        ensure!(
            cycles.len() == clocks as usize + load_handoff_cycles,
            "incomplete cycle log; observation saved"
        );
        let signature = (observation.result, observation.final_memory);
        if let Some(prior) = &previous {
            ensure!(
                prior == &signature,
                "{} changed between captures; both observations saved",
                case.name
            );
        }
        previous = Some(signature);
    }
    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();
    let args = Args::parse();
    let cases: Vec<_> = cases(args.cpu)
        .into_iter()
        .filter(|case| args.filter.as_ref().is_none_or(|filter| case.name.contains(filter)))
        .collect();
    ensure!(!cases.is_empty(), "no cases match the filter");
    if args.list {
        for case in &cases {
            println!("{}", case.name);
        }
        return Ok(());
    }
    if !args.assemble_only {
        ensure!(
            args.chip.as_ref().is_some_and(|s| !s.trim().is_empty()),
            "capture requires --chip markings"
        );
        ensure!(
            args.board.as_ref().is_some_and(|s| !s.trim().is_empty()),
            "capture requires --board and firmware details"
        );
    }
    let output = args.output.as_ref().unwrap();
    ensure!(
        !output.exists(),
        "output directory already exists; choose a new capture directory"
    );
    if let Some(parent) = output.parent().filter(|parent| !parent.as_os_str().is_empty()) {
        fs::create_dir_all(parent)?;
    }
    fs::create_dir(output)?;
    fs::copy(SOURCE, output.join("segment_limits.asm"))?;
    // Assemble everything before connecting, so an assembler failure cannot
    // interrupt an otherwise valid hardware capture.
    let mut fixtures = Vec::new();
    for case in &cases {
        let dir = output.join(&case.name);
        fs::create_dir(&dir)?;
        let fixture = assemble(case, &args.nasm, &dir.join("program.bin"))?;
        let regs = initial_registers(case.cpu, fixture.entry)?;
        fs::write(dir.join("registers.bin"), &regs)?;
        fs::write(dir.join("case.toml"), toml::to_string_pretty(case)?)?;
        fixtures.push((fixture, regs, dir));
    }
    println!("Assembled {} cases in {}", cases.len(), output.display());
    if args.assemble_only {
        return Ok(());
    }
    let mut client = CpuClient::init(args.com_port.clone(), Some(5000))?;
    let (cpu, _) = client.cpu_type()?;
    ensure!(
        matches!(
            (args.cpu, cpu),
            (Cpu::I286, ServerCpuType::Intel80286) | (Cpu::I386, ServerCpuType::Intel80386)
        ),
        "requested {:?}, connected to {cpu}",
        args.cpu
    );
    let old_flags = client.get_flags()?;
    let run = (|| -> Result<()> {
        client.set_flags(
            ServerFlags::USE_SDRAM_BACKEND | ServerFlags::RESOLVE_BUS_STEP | ServerFlags::ENABLE_CYCLE_LOGGING,
        )?;
        for (case, (fixture, regs, dir)) in cases.iter().zip(&fixtures) {
            println!("Capturing {}", case.name);
            capture(&args, &mut client, case, fixture, regs, dir).with_context(|| case.name.clone())?;
        }
        Ok(())
    })();
    // Never restore automatic execution while a CPU is parked in protected mode.
    let restore = client.set_flags(old_flags & !ServerFlags::EXECUTE_AUTOMATIC);
    run?;
    restore?;
    Ok(())
}

#[cfg(test)]
#[path = "segment_limits/smoke.rs"]
mod smoke;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matrix_has_unique_names_and_deliverable_stack_frames() {
        for cpu in [Cpu::I286, Cpu::I386] {
            let cases = cases(cpu);
            let names: BTreeSet<_> = cases.iter().map(|case| &case.name).collect();
            assert_eq!(cases.len(), names.len());
            for case in cases {
                if cpu == Cpu::I286 {
                    assert!(!case.big && !case.address32 && case.width <= 2 && case.offset <= 0xffff);
                }
                if case.stack && case.protected {
                    assert!(if case.down {
                        case.limit < 0x7ff4
                    }
                    else {
                        case.limit >= 0x8000
                    });
                }
                assert!(input_memory(&case).iter().all(|m| m.address < 0x100000));
            }
        }
    }

    #[test]
    fn matrix_covers_zero_lower_limit_and_maximum_upper_limit() {
        for cpu in [Cpu::I286, Cpu::I386] {
            let cases = cases(cpu);
            for stack in [false, true] {
                for write in [false, true] {
                    assert!(cases.iter().any(|c| c.protected
                        && c.down
                        && c.limit == 0
                        && c.offset == 0
                        && c.stack == stack
                        && c.write == write));
                    assert!(cases.iter().any(|c| !c.down
                        && c.limit == 0xffff
                        && c.offset == 0xffff
                        && c.width == 2
                        && c.stack == stack
                        && c.write == write));
                }
            }
        }
        assert!(cases(Cpu::I386)
            .iter()
            .any(|c| c.limit == u32::MAX && c.offset == u32::MAX && c.width == 4));
    }

    #[test]
    fn malformed_results_and_fixtures_are_rejected() {
        assert!(outcome(&[0; 27]).is_err());
        assert!(Fixture::parse(Vec::new()).is_err());
        assert!(!outcome(&[0; RESULT_SIZE]).unwrap().complete);
    }

    #[test]
    fn observation_preserves_inputs_and_raw_result_values() {
        let case = cases(Cpu::I386).into_iter().find(|c| c.limit == u32::MAX).unwrap();
        let observation = Observation {
            format_version: 1,
            chip: "example chip",
            board: "example board and firmware",
            case: &case,
            attempt: 1,
            termination: "timeout".to_owned(),
            clocks: 256,
            load_handoff_cycles: 1,
            logged_cycles: 256,
            probe_ip: 42,
            initial_memory: input_memory(&case),
            final_memory: input_memory(&case),
            result: outcome(&[0xff; RESULT_SIZE]).unwrap(),
        };
        let text = toml::to_string_pretty(&observation).unwrap();
        let parsed: toml::Value = toml::from_str(&text).unwrap();
        assert_eq!(parsed["case"]["limit"].as_integer(), Some(i64::from(u32::MAX)));
        assert_eq!(parsed["result"]["value"].as_integer(), Some(i64::from(u32::MAX)));
        assert_eq!(parsed["result"]["complete"].as_bool(), Some(false));
        assert_eq!(
            parsed["initial_memory"].as_array().unwrap().len(),
            observation.initial_memory.len()
        );
    }

    #[test]
    fn loadall_buffers_match_client_register_formats() {
        for cpu in [Cpu::I286, Cpu::I386] {
            let bytes = initial_registers(cpu, 18).unwrap();
            assert_eq!(bytes.len(), RegisterSetType::from(cpu.server()).size());
            let regs = arduinox86_client::RemoteCpuRegisters::try_from(bytes.as_slice()).unwrap();
            assert_eq!(regs.code_address(), CODE_BASE + 18);
        }
    }
}
