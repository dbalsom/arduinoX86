// Main user config. You may wish to edit things here to suit your board and CPU.

// Define this if you are using the Display Shield for the Arduino Giga.
// Warning: greatly increases compile time.
#define GIGA_DISPLAY_SHIELD 0
#define SCREEN_UPDATE_FPS 30
#define SCREEN_UPDATE_MS (1000 / SCREEN_UPDATE_FPS)

// Define this if you have connected an 8087 FPU
//#define FPU_8087

// The main AruduinoX86 shields are:
// SHIELD_8088_V1 - Shield8088 rev 1.1 - Due & Giga compatible, supporting the 8088, 8086, NEC V20 and NEC V30 CPUs
// SHIELD_80186_3V_V1 - Shield80186 3V rev 1 - Due & Giga compatible, supporting 3V 186 CPUs (80L186)
// SHIELD_286_3V_V1 - Shield286_3V - Due & Giga compatible. Uses shifters to operate at 3V. (Never produced)
// SHIELD_286_5V_V1 - Shield286_5V - Giga only, allowing 5V 286 CPUs (80C286) to be used without shifters.

// Only define one of these!
//#define SHIELD_8088_V1
//#define SHIELD_80186_3V_V1
#define SHIELD_286_5V_V1
//#define SHIELD_386EX_V1
//#define SHIELD_386EX_V2

#if (defined(SHIELD_8088_V1) + defined(SHIELD_80186_3V_V1) + defined(SHIELD_286_5V_V1) + defined(SHIELD_386EX_V1) + defined(SHIELD_386EX_V2)) != 1
  #error "You must define only one shield type!"
#endif

// If using 286 shield, specify whether you are using an 82284 clock generator chip.
#define USE_82284

// Baud rate is ignored for Arduino DUE as it uses native SerialUSB. This is legacy.
// YOU SHOULD BE USING A DUE.
//
// For Arduino MEGA, Arduino-branded MEGAs should use 460800 baud. ELEGOO branded MEGAs 
// can use 1000000. 
// You can test higher values but these are the values I determined to work without errors.
// Actual limits may be board-specific!
//#define BAUD_RATE 460800
#define BAUD_RATE 1000000

// DEBUG_BAUD_RATE controls the Serial1 speed. Check the documentation of your RS232 interface
// for maximum rated speed. Exceeding it will cause dropped characters or other corruption.
// The popular MAX3232 module has a maximum rate of 250Kbps, so should use a baud rate of 230400.
// The TRS3122E module specified in the BOM can support 1Mbit. I have been using 460800 with it.
#define DEBUG_BAUD_RATE 460800

#define CMD_TIMEOUT 100 // Command timeout in milliseconds
#define MAX_ERROR_CYCLES 5
#define EXECUTE_TIMEOUT 1000
// What vector to use for the BRKEM call. No reason to change this really.
#define BRKEM_VECTOR ((uint8_t)0x00)

#define SILENT_MODE 0 // If set, disables all traces and debugs.

// Print a character to the debugging output on each load command.
#define LOAD_INDICATOR (SILENT_MODE)
// Print a character to the debugging output on each store command.
#define STORE_INDICATOR (SILENT_MODE)

#define TRACE_ALL 0 // TRACE_ALL will enable all traces (TRACE_NONE overrides)
#define TRACE_NONE (0 | SILENT_MODE) // TRACE_NONE will override all set traces

// These defines control tracing and debugging output for each state.
// Note: tracing a STORE operation will likely cause it to timeout on the client.
#define TRACE_RESET     ((1 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces during CPU Reset.
#define TRACE_SETUP     ((1 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces for the CpuSetup state.
#define TRACE_VECTOR    ((1 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces for the JumpVector state.
#define TRACE_LOAD      ((1 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces for the Load state.
#define TRACE_ID        ((1 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces for the CpuId state.
#define TRACE_PREFTECH  ((0 | TRACE_ALL) & ~TRACE_NONE) // Print cycle traces for the Prefetch state.
#define TRACE_EMU_ENTER ((0 | TRACE_ALL) & ~TRACE_NONE)
#define TRACE_EMU_EXIT  ((0 | TRACE_ALL) & ~TRACE_NONE)
#define TRACE_EXECUTE   ((1 | TRACE_ALL) & ~TRACE_NONE)
#define TRACE_STORE     ((1 | TRACE_ALL) & ~TRACE_NONE)
#define TRACE_FINALIZE  ((1 | TRACE_ALL) & ~TRACE_NONE)

#define DEBUG_ALL 0  // DEBUG_ALL will enable all debugs (DEBUG_NONE overrides)
#define DEBUG_NONE (0 | SILENT_MODE) // DEBUG_NONE will override all set debugs

#define DEBUG_SERVER    ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about the server state
#define DEBUG_STATE     ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about state changes and time spent in each state
#define DEBUG_RESET     ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about the reset process
#define DEBUG_SETUP     ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about the CPU setup routine, if applicable
#define DEBUG_VECTOR    ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about jump vector program execution
#define DEBUG_ID        ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about CPU identification
#define DEBUG_LOAD      ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about events during LOAD state
#define DEBUG_LOAD_DONE ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about events during LOAD_DONE state
#define DEBUG_EXECUTE   ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about events during EXECUTE state
#define DEBUG_STORE     ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about events during STORE state
#define DEBUG_FINALIZE  ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about events during FINALIZE state
#define DEBUG_INSTR     ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Print instruction mnemonics as they are executed from queue
#define DEBUG_EMU       ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about 8080 emulation mode state
#define DEBUG_QUEUE     ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about queue operations (flushes, regular queue ops are always reported)
#define DEBUG_TSTATE    ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about t-state changes (mostly T3/Tw->T4)
#define DEBUG_PIN_CMD   ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about pin write commands
#define DEBUG_BUS       ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about bus parameters (Width, etc), writes (cmd_write_data_bus)
#define DEBUG_PROTO     ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about the serial cpu_server protocol (verbose)
#define DEBUG_CMD       ((1 | DEBUG_ALL) & ~DEBUG_NONE) // Info about command processing and dispatch
#define DEBUG_DUMP      ((0 | DEBUG_ALL) & ~DEBUG_NONE) // Info about dump commands

#define MAX_ERR_LEN 50 // Maximum length of an error string

#define FINALIZE_TIMEOUT 30
#define FINALIZE_EMU_TIMEOUT 90 // We need more time to exit emulation mode

