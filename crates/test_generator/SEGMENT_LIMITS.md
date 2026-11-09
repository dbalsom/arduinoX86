# Segment limit captures

`segment_limits` is a second binary in the existing `test_generator` package.
It uses `arduinox86_client`, the server's LOADALL support, SDRAM memory backend
and cycle logger. No firmware or serial protocol changes are needed. The normal
`test_generator` command and its MOO output are unchanged.

These are deterministic program captures for gaps in the existing real mode
instruction corpus. In particular, that corpus cannot establish what happens
with a zero limit in a protected mode expand down segment. A maximum limit also
needs to be distinguished from a limit just below it, on identified chips.
The tool records what happened without declaring a disputed boundary behavior
correct or incorrect.

## Coverage

Each fixture runs one MOV through DS or SS. Both ModR/M and direct memory
(moffs) encodings are included, for reads and writes. Cases cover:

* Real mode offsets at and across the 64 KiB boundary.
* Protected mode expand up and expand down descriptors with limits 0, 1,
  FFFEh and FFFFh. There are accesses at the limit, just above it and at the
  last position where the operand fits, as well as accesses crossing the end.
* Byte and word operands on the 286, with dword operands on the 386EX.
* Both address sizes and both descriptor B values on the 386EX, including
  offsets above FFFFh for an expand down segment with B=0 versus B=1.
* A 386EX limit of FFFFEFFFh versus FFFFFFFFh, including operands crossing
  the 32 bit offset boundary. Both limits use page granularity.
* Empty expand down segments and ordinary zero limit segments through DS.

SS cases are included only where the exception handler's stack frame is valid.
The handler uses the same privilege level and a 16 bit interrupt gate. Empty or
tiny SS segments would require a separate task or privilege stack to distinguish
the operand fault from a failure during exception delivery. Those combinations,
implicit PUSH/POP behavior, task switches and NMI behavior are outside this suite.
The server currently identifies 286 and 386 CPUs. This does not add 486 hardware
support or treat 386EX observations as proof about other 386/486 parts.

## Build and run

Install NASM and build with the workspace's usual Cargo tools. Start with a
small filter before attempting the complete matrix:

```sh
cargo run -p test_generator --bin segment_limits -- --cpu 286 --list

cargo run -p test_generator --bin segment_limits -- \
  --cpu 286 --assemble-only --output target/segment-fixtures-286

cargo run -p test_generator --bin segment_limits -- \
  --cpu 286 --com-port /dev/ttyACM0 \
  --chip 'manufacturer, complete chip markings and revision' \
  --board 'board/shield revision and firmware commit' \
  --filter pm-down-b0-l00000000-o00000000 \
  --output target/segment-capture-286
```

Use `--cpu 386` with a supported 386EX shield. This runner requires a server
with the SDRAM backend, stepped bus resolution and cycle logging. `--nasm`
selects a NASM executable. `--filter` is a substring match against `--list`.
Each output directory must be new, so earlier captures cannot be overwritten.
Include the directory and the source commit when sharing results. Do not commit
generated binaries or claim emulator output as a physical CPU capture.

Each attempt resets the CPU through the existing register load command and
clears server memory. The default is two attempts per case. Differences in the
recorded outcome or memory stop the run and preserve both observations.
`--repeat`, `--max-cycles` and `--timeout-seconds` adjust these bounds. The clock
is stepped through the existing serial client, so a full run can take some time.
Use this on an otherwise idle test board.

The program enters protected mode itself, with ordinary GDT and IDT entries.
It writes a result record and jumps to a separate rendezvous loop. The host
stops stepping on the first fetch there. It does not use automatic real mode
exception detection or HLT/NMI register readout for protected mode completion.
Cycle logs are drained every 256 clocks to avoid the server's ring buffer
wrapping. Automatic execution remains disabled when the runner exits.

## Reading a capture

Each case directory contains the assembled `program.bin`, the LOADALL
`registers.bin`, and a `case.toml` specifying all descriptor and operand inputs.
The root also contains the assembly source used to generate them. Each attempt
adds:

* `N.toml`: chip and board details, completion reason, the result record, and
  memory before and after execution.
* `N.result.bin`: the unmodified 28 byte result record from the program.
* `N.cycles.csv`: raw server cycle fields, including address/data buses and
  ALE, BHE, READY and LOCK pin bits. Values other than the cycle index are hex.

`vector = 65535` means the operand completed without entering a handler.
`error_code = 65535` means no error code was pushed. A fault record includes
the original saved IP, CS and FLAGS. `probe_ip` identifies the instruction under
test, so a setup fault cannot silently be reported as a boundary result. The
record also contains AX/EAX, the stack pointer before and after the attempt,
and MSW. AX is zero extended in the 286 record. Successful completion is
independent of whether the operand faulted.

Memory snapshots include guards around the operand, the segment base, and the
address obtained by truncating the offset to 16 bits. This exposes partial
writes and distinguishes wrapping from a contiguous access. All observed memory
is below 1 MiB. The large offset cases add a base of 20000h and wrap the linear
address before reaching the external bus. They do not require capturing address
pins above those available on the existing shields.

These program observations deliberately are not written as single instruction
MOO tests. The current real mode register conversion and IVT exception decoder
do not describe their protected mode setup or handled exceptions. The explicit
descriptor inputs, program bytes and raw cycles are retained for inspection and
for future protected mode corpus support.

Timeouts, server errors, faults outside the probe, missing cycles and inconsistent
repetitions fail the command. Partial observations are saved when the serial
connection still permits readback. A communication failure may prevent readback
and is not a completed measurement. Neither completion nor agreement between
repetitions is an architectural correctness verdict.

## Local validation

```sh
cargo test -p test_generator
cargo test -p test_generator --bin segment_limits \
  fixture_matrix_assembles -- --ignored --nocapture
cargo test -p test_generator --bin segment_limits \
  fixture_execution_and_fault_records -- --ignored --nocapture
```

The second command needs NASM. It assembles and decodes all 376 286 fixtures and
2644 386 fixtures, checking the operand, segment, address size and encoding.
The third command also needs `qemu-system-i386`. It boots selected
fixtures in QEMU and checks their records, register preservation, successful
reads and exception returns. It also deliberately causes #UD, #GP and #SS to
exercise frames with and without an error code. It uses QEMU's 486 model to
validate the test code, including the code assembled for the 286. It does not
validate 286 timing or silicon behavior. Temporary images are removed on success
and retained with a diagnostic path on failure. `SEGMENT_TEST_NASM` and
`SEGMENT_TEST_QEMU` can select the tool executables.
