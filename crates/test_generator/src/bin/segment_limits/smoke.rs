//! Check fixture plumbing in QEMU. This is not evidence about physical CPUs.
use super::*;
use std::{env, process::Stdio, thread};

#[test]
#[ignore = "requires NASM; assembles and decodes every fixture without hardware"]
fn fixture_matrix_assembles() -> Result<()> {
    use iced_x86::{Decoder, DecoderOptions, Mnemonic, OpKind, Register};
    let nasm = PathBuf::from(env::var_os("SEGMENT_TEST_NASM").unwrap_or_else(|| "nasm".into()));
    let tmp = tempfile::tempdir()?;
    for cpu in [Cpu::I286, Cpu::I386] {
        let cases = cases(cpu);
        for case in &cases {
            let fixture = assemble(case, &nasm, &tmp.path().join("program.bin"))?;
            let bytes = &fixture.bytes[usize::from(fixture.probe)..];
            let instruction = Decoder::new(16, bytes, DecoderOptions::NONE).decode();
            ensure!(
                instruction.mnemonic() == Mnemonic::Mov,
                "wrong probe instruction: {}",
                case.name
            );
            ensure!(
                instruction.memory_size().size() == usize::from(case.width),
                "wrong operand width: {}",
                case.name
            );
            ensure!(
                instruction.op0_kind() == if case.write { OpKind::Memory } else { OpKind::Register },
                "wrong access direction: {}",
                case.name
            );
            ensure!(
                instruction.memory_segment() == if case.stack { Register::SS } else { Register::DS },
                "wrong segment: {}",
                case.name
            );
            ensure!(
                usize::from(fixture.probe) + instruction.len() == usize::from(word(&fixture.bytes, 10)),
                "probe contains more than one instruction: {}",
                case.name
            );
            if case.direct {
                ensure!(
                    (0xa0..=0xa3).contains(&instruction.op_code().op_code()),
                    "wrong direct encoding"
                );
                ensure!(
                    instruction.memory_displacement64() == u64::from(case.offset),
                    "wrong displacement"
                );
                ensure!(
                    instruction.memory_displ_size() == if case.address32 { 4 } else { 2 },
                    "wrong address size"
                );
            }
            else {
                ensure!(
                    instruction.memory_base() == if case.address32 { Register::EBX } else { Register::BX },
                    "wrong address size: {}",
                    case.name
                );
            }
        }
        println!("Assembled and decoded {} {} fixtures", cases.len(), cpu.number());
    }
    Ok(())
}

#[test]
#[ignore = "requires NASM and qemu-system-i386; runs emulator smoke tests, not silicon measurements"]
fn fixture_execution_and_fault_records() -> Result<()> {
    let tmp = tempfile::tempdir()?;
    match run_fixtures(tmp.path()) {
        Ok(()) => Ok(()),
        Err(error) => Err(error.context(format!("smoke artifacts retained in {}", tmp.keep().display()))),
    }
}

fn run_fixtures(tmp: &Path) -> Result<()> {
    let nasm = env::var_os("SEGMENT_TEST_NASM").unwrap_or_else(|| "nasm".into());
    let qemu = env::var_os("SEGMENT_TEST_QEMU").unwrap_or_else(|| "qemu-system-i386".into());
    let mut count = 0;
    for cpu in [Cpu::I286, Cpu::I386] {
        let selected = cases(cpu).into_iter().filter(|case| {
            let basic = case.width == 2
                && case.address32 == (cpu == Cpu::I386)
                && case.big == (case.protected && cpu == Cpu::I386)
                && case.down == case.protected
                && case.limit == if case.protected { 0 } else { 0xffff }
                && case.offset == if case.protected { 0 } else { 0xffff };
            let wide =
                cpu == Cpu::I386 && !case.down && case.limit == u32::MAX && case.offset == u32::MAX && case.width == 4;
            let valid = case.protected
                && case.down
                && case.limit == 0
                && case.offset == 1
                && case.width == 1
                && !case.address32
                && !case.big;
            basic || wide || valid
        });
        for case in selected {
            for force_fault in [0, 6, 12, 13] {
                if !case.protected && force_fault > 6 {
                    continue;
                }
                let bin = tmp.join("program.bin");
                let wrapper = tmp.join("smoke.asm");
                let mut source = String::from("%define SMOKE 1\n%macro SMOKE_SETUP 0\n");
                for memory in input_memory(&case) {
                    source.push_str(&format!(
                        "mov ax, {}\nmov ds, ax\nmov byte [{}], {}\n",
                        memory.address >> 4,
                        memory.address & 15,
                        memory.bytes[0]
                    ));
                }
                source.push_str("%endmacro\n");
                if force_fault != 0 {
                    source.push_str(&format!("%define SMOKE_FAULT {force_fault}\n"));
                }
                source.push_str(&format!("%include \"{SOURCE}\"\n"));
                fs::write(&wrapper, source)?;
                let mut command = Command::new(&nasm);
                command.args(["-f", "bin", "-Werror"]).arg(&wrapper).arg("-o").arg(&bin);
                configure_assembler(&case, &mut command);
                let assembled = command.output()?;
                ensure!(
                    assembled.status.success(),
                    "{}",
                    String::from_utf8_lossy(&assembled.stderr)
                );
                let fixture = Fixture::parse(fs::read(&bin)?)?;
                let sectors = fixture.bytes.len().div_ceil(512);
                ensure!(sectors < 18, "fixture exceeds one floppy track");

                let boot = tmp.join("boot.asm");
                fs::write(
                    &boot,
                    format!(
                        r#"
bits 16
org 0x7c00
    cli
    xor ax, ax
    mov ss, ax
    mov sp, 0x7c00
    mov ds, ax
    sti
    mov ax, 0x1000
    mov es, ax
    xor bx, bx
    mov ax, 0x0200 | {sectors}
    mov cx, 2
    xor dh, dh
    int 0x13
    jc failed
    jmp 0x1000:{entry}
failed:
    mov al, 1
    out 0xf4, al
    jmp failed
    times 510-($-$$) db 0
    dw 0xaa55
"#,
                        entry = fixture.entry
                    ),
                )?;
                let image = tmp.join("boot.img");
                let assembled = Command::new(&nasm)
                    .args(["-f", "bin"])
                    .arg(&boot)
                    .arg("-o")
                    .arg(&image)
                    .output()?;
                ensure!(assembled.status.success(), "boot sector assembly failed");
                let mut disk = fs::OpenOptions::new().append(true).open(&image)?;
                disk.write_all(&fixture.bytes)?;
                disk.set_len(1440 * 1024)?;
                drop(disk);
                let result_file = tmp.join("result.bin");
                let mut child = Command::new(&qemu)
                    .args([
                        "-machine",
                        "pc",
                        "-accel",
                        "tcg",
                        "-cpu",
                        "486",
                        "-m",
                        "4M",
                        "-display",
                        "none",
                        "-monitor",
                        "none",
                        "-serial",
                        "none",
                        "-no-reboot",
                        "-boot",
                        "a",
                        "-device",
                        "isa-debug-exit,iobase=0xf4,iosize=0x04",
                        "-global",
                        "isa-debugcon.iobase=0xe9",
                    ])
                    .arg("-drive")
                    .arg(format!("file={},format=raw,if=floppy", image.display()))
                    .arg("-debugcon")
                    .arg(format!("file:{}", result_file.display()))
                    .args(["-d", "guest_errors,int,cpu_reset", "-D"])
                    .arg(tmp.join("qemu.log"))
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .spawn()?;
                let start = Instant::now();
                let status = loop {
                    if let Some(status) = child.try_wait()? {
                        break status;
                    }
                    if start.elapsed() > Duration::from_secs(10) {
                        child.kill()?;
                        child.wait()?;
                        anyhow::bail!("QEMU timed out: {} (force_fault={force_fault})", case.name);
                    }
                    thread::sleep(Duration::from_millis(10));
                };
                ensure!(status.code() == Some(1), "QEMU failed: {}", case.name);
                let result = outcome(&fs::read(&result_file)?)?;
                ensure!(result.complete, "incomplete result: {}", case.name);
                ensure!(
                    result.stack_before == 0x8000 && result.stack_after == 0x8000,
                    "stack was not restored: {}: {result:?}",
                    case.name
                );
                ensure!(
                    result.msw & 1 == u16::from(case.protected),
                    "wrong CPU mode: {}",
                    case.name
                );
                let sentinel = if cpu == Cpu::I286 { 0x9e71 } else { 0xa5c39e71 };
                if result.vector != 0xffff {
                    ensure!(
                        result.saved_ip == fixture.probe,
                        "wrong saved IP: {}: {result:?}",
                        case.name
                    );
                    ensure!(
                        result.saved_cs == if case.protected { 8 } else { 0x1000 },
                        "wrong saved CS"
                    );
                    ensure!(result.value == sentinel, "fault changed operand register");
                }
                else if !case.write {
                    let memory = input_memory(&case);
                    let address = DATA_BASE.wrapping_add(case.offset);
                    let mut expected = sentinel;
                    for i in 0..u32::from(case.width) {
                        let byte = memory.iter().find(|m| m.address == address + i).unwrap().bytes[0];
                        expected = (expected & !(0xff << (i * 8))) | (u32::from(byte) << (i * 8));
                    }
                    ensure!(result.value == expected, "wrong read value: {}: {result:?}", case.name);
                }
                else {
                    ensure!(result.value == sentinel, "write changed operand register");
                }
                if force_fault != 0 {
                    let error = match force_fault {
                        6 => 0xffff,
                        12 => 40,
                        13 => 0x9e70,
                        _ => unreachable!(),
                    };
                    ensure!(
                        result.vector == force_fault && result.error_code == error,
                        "forced exception was not recorded correctly: {result:?}"
                    );
                }
                count += 1;
            }
        }
    }
    println!("{count} fixture/handler smoke runs completed in QEMU (not silicon evidence)");
    Ok(())
}
