![arduinox86_logo](/images/arduinox86_logo_transparent_01.png)

### About ArduinoX86

I've written a blog article that gives an overview of this project and how it is used.

https://martypc.blogspot.com/2023/06/hardware-validating-emulator.html

This project was originally called Arduino8088, but then I expanded it to support the 8086, 80186, 80286, with plans for
the 80386, so it is now named the suitably generic ArduinoX86.

### Description

This project expands on the basic idea of controlling a CPU via GPIO pins to clock the CPU and read and write control
and data signals. This can be used to validate an emulator's accuracy, but also as a general method of exploring the
operation of CPU instructions and timings.

This project currently supports the following CPUs:

- CMOS variants of the 8088 (Oki, Harris 80C88, etc.)
- AMD D8088
- CMOS variants of the 8086 (Intel 80C86, Harris 80C86)
- NEC V20
- NEC V30
- Intel 80L186
- Harris 80C286
- Intel 80386EX

Where it differs from existing Raspberry Pi-based projects is that it uses an Arduino Giga for the expanded number of
GPIO pins available. The Giga has enough GPIO to operate 808X-compatible CPUs in Maximum mode. This enables several
useful signals to be read such as the QS0 & QS1 processor instruction queue status lines, which give us more insight
into the internal state of the CPU. We can also enable inputs such as READY, NMI, INTR, and TEST, so we can execute
interrupts, NMIs, emulate wait states, and potentially even simulate FPU and DMA operations.

I have been using this project to validate the cycle-accuracy of my PC
emulator, [MartyPC](https://github.com/dbalsom/martypc), and produce CPU test
suites for emulators, covering
the [8088](https://github.com/singleStepTests/8088/), [V20](https://github.com/SingleStepTests/v20)
and [8086](https://github.com/singleStepTests/8086/) and [286](https://github.com/singlesteptests/80286) to date.

This project has three main components:

- The CPU server software, which runs on the Arduino, and establishes a serial protocol to reset the CPU, load and store
  register state, as well as interactively operate the CPU if desired;
- The CPU client software, which runs on your computer and controls the CPU after the CPU server has set up initial
  register state as desired, or uploads an entire program to be run automatically with register state collected at the
  end of execution;
- The CPU socket PCB shields, which seat on top of the Arduino, and provide a physical interface between the CPU and
  GPIO
  lines.

The original "Arduino8088" project utilized an Arduino MEGA, but this board no longer supported.
It is painfully slow and limited to a serial UART instead of fast native USB communication. As of the current
release, an [Arduino Due](https://store.arduino.cc/products/arduino-due), or preferably,
an [Arduino GIGA](https://store.arduino.cc/products/giga-r1-wifi) should be used instead. A GIGA is required to
operate 5V CPUs such as older 80186's and the 80286. Despite Arduino's official warnings, the STM32 GPIO in the Giga is
5V tolerant (with a few exceptions). You will likely void your warranty and I take no responsibility for any damage to
your Arduino that occurs attempting to use a 5V CPU with your Arduino.

### 8088,8086,V20,V30 Support

Revision 1.1 of the 808X shield supports the 8086 and NEC V30 by adding a connection for the 8086's BHE pin.

The Due can operate the 8088, 8086, V20 and V30, despite these chips being 5V. The current 8088 shield feeds those CPUs
3.3V which they tolerate quite well at low clock speeds.

The Giga can operate the 8088, 8086, V20 and V30 directly at 5V if desired. The Giga requires the 3V supply pin to be
cut off the shield's pin headers and external power supplied due to the lower available overall power budget.

> [!WARNING]  
> Do not attempt to use the 8088 shield with a Giga without using external power, or you may damage your Arduino.

### 80186 Support

The ArduinoX86 cpu_server sketch supports operation of a low-voltage 80L186 on an Arduino Due.

There is no shield yet for the 80186, you will need to use a breakout board and connect directly to the Arduino Due's
headers. See [Using an 80186](https://github.com/dbalsom/arduinoX86/wiki/Using-an-80186) for more information.

<img width="349" height="620" alt="image" src="https://github.com/user-attachments/assets/c7e5fa0e-8c6f-4119-a0f4-7b936193ac5e" />

### 80286 Support

A shield for the 80C286 is in the design phase, located in `/shields/286_5V`

Current support for the 80286 has been performed via breadboard.

<img width="800" height="480" alt="image" src="https://github.com/user-attachments/assets/2d5ffa03-ebcf-48c8-bfe2-1574fdb0de2b" />

### 80386 Support

A shield for the 80386EX CPU is provided in `/shields/386EX_V4`

![386ex_shield_v4](/shields/386EX_V4/images/386ex_v4_render_01.png)

## Can the CPU be clocked fast enough with an Arduino?

In short, no. We are well past the published minimum cycle times when executing programs via a serial protocol, cycle by
cycle. Some chips tolerate this better than others. When working with an Intel branded 8088, I noticed that effective
address calculations were failing to add the displacement or index register, but otherwise functioned. I have had more
luck with the AMD second-source 8088 CPUs, which seem to function perfectly under slow clocks, although they will hang
and need to be reset if not cycled for a several milliseconds. The issue is "dynamic logic" - logic gates that lose
their state if not refreshed electrically within a frequent enough interval. To be absolutely safe, it is best to use a
fully CMOS process CPU such as the 80C88. CMOS versions of most x86 chips are available.

## To use

An example application for cpu_server is provided, written in Rust, in the `/crates/exec_program` directory. It
demonstrates how to upload arbitrary code to the ArduinoX86 and display cycle traces. The client will emulate the
entire address space and set up a basic IVT.

# Project Structure

## /asm

Assembly language files, intended to be assembled with NASM. To execute code on the ArduinoX86, one must supply two
binary files, one containing the program to be executed, and one containing the register values to load onto the CPU
before program execution.

## /crates/arduinox86_client

A library crate that implements a client for the ArduinoX86's serial protocol.

## /crates/arduinox86_cpu

A library crate built on top of the `arduinox86_client` crate, this provides a `RemoteCpu` struct that models CPU state
and can execute programs.

## /crates/arduinox86_gui

A GUI (written in egui) for ArduinoX86, supporting the 386EX. It allows you to write and assemble assembly-language
programs and execute them on the CPU with an easy-to-use interface.

## /crates/exec_program

A binary implementing an interface for the `arduinox86_cpu` crate that will load a provided register state binary and
execute the specified program binary.

## /crates/test_generator

A program that generates CPU tests for emulator authors.

The [segment limit capture suite](crates/test_generator/SEGMENT_LIMITS.md) adds deterministic
286 and 386EX real and protected mode observations using the same CPU client and server.

### /shields

Contains the KiCad project files and Gerber files for the various ArduinoX86 shields. See the README.md in each shield
directory for more information and BOMs for building each.

### /platformio/ArduinoX86

The main server code that runs on the Arduino Due or GIGA. You must have [platformio](https://platformio.org/) installed
to build and upload the project to your Arduino.

## Archived Directories

### /sketches/cpu_server

The original cpu_server Arduino IDE sketch. Development has since moved to platformio.

### /sketches/run_program

The original sketch for Arduino MEGA only demonstrating how to control an 8088. It does not contain a serial protocol
server - code is run directly from an array defined within the sketch.


# Ordering Boards

- Ordering boards from any of the KiCad projects is extremely simple using PCBWay's KiCad fabrication plugin.
  - From the Plugin and Content Manager, go the Fabrication Plugins tab, find the PCBWay plugin and install.
  - <img width="620" height="405" alt="image" src="https://github.com/user-attachments/assets/c1e08ee9-d995-4615-a5df-647f5c32cd7b" />
  - Open the PCB editor and click the PCBWay Button
  - <img width="192" height="115" alt="image" src="https://github.com/user-attachments/assets/50aeec0d-4424-4750-b5c3-ba1a5c47c763" />
  - A browser window will open to PCBWay's website with the project uploaded and acceptable defaults selected.
  - If you like you can change the board color, or get a nicer finish with gold (ENIG).
  - <img width="541" height="51" alt="image" src="https://github.com/user-attachments/assets/41317565-d556-463a-9d48-33a104033afe" />
  - You can then proceed to ordering. Part of your purchase goes back to help fund the KiCad project, which is cool.
  - <img width="333" height="65" alt="image" src="https://github.com/user-attachments/assets/292e06a9-254a-4270-a2c4-a197c8c89bd7" />

    
# Credits

Inspired by the Pi8088 validator created by Andreas Jonsson as part of the VirtualXT project:

https://github.com/andreas-jonsson/virtualxt/tree/develop/tools/validator/pi8088

# Other Cool Projects

- 8bitforce's Retroshield: https://www.8bitforce.com/projects/retroshield/

- homebrew8088's Raspberry Pi Hat: https://github.com/homebrew8088/pi86

- The Universal Chip Analyser (U. C. A.): https://x86.fr/uca/

- Foxtech's 486 Breadboard Computer: https://www.youtube.com/watch?v=wSiDSdHS2QQ

