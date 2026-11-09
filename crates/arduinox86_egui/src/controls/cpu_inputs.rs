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
use crate::{
    events::{GuiEvent, GuiEventQueue},
    serial_worker::PinReadout,
};
use arduinox86_client::{CpuClientError, CpuPin, ServerCpuType};

/// Checkbox values are refreshed from the hardware. READY is a logical signal;
/// the legacy TEST/BUSY command still reports the physical pin level.
pub struct CpuInputControls {
    pins: [InputPin; 3],
}

struct InputPin {
    pin: CpuPin,
    label: &'static str,
    tooltip: &'static str,
    inverted: bool,
    value: Option<bool>,
    unavailable: Option<&'static str>,
}

impl InputPin {
    fn read(&mut self, result: Result<bool, CpuClientError>) {
        match result {
            Ok(value) => self.value = Some(value ^ self.inverted),
            Err(error) => {
                self.value = None;
                if matches!(error, CpuClientError::CommandFailed(_)) {
                    // Older firmware rejects HOLD. Remember this until reconnect
                    // instead of repeatedly sending an unsupported command.
                    self.unavailable = Some("This pin is not supported by the connected firmware/shield.");
                }
                log::warn!("Failed to read {}: {}", self.label, error);
            }
        }
    }
}

impl Default for CpuInputControls {
    fn default() -> Self {
        Self::new(ServerCpuType::Undetected)
    }
}

impl CpuInputControls {
    pub fn new(cpu: ServerCpuType) -> Self {
        let is_286 = matches!(cpu, ServerCpuType::Intel80286);
        let is_386 = matches!(cpu, ServerCpuType::Intel80386);
        let unknown = matches!(cpu, ServerCpuType::Undetected).then_some("CPU type has not been detected.");
        Self {
            pins: [
                InputPin {
                    pin: CpuPin::READY,
                    label: "READY",
                    tooltip: "Checked: READY asserted. Unchecked: insert wait states. Firmware may change READY while running or resetting.",
                    // READY is always active-high in the client protocol.
                    inverted: false,
                    value: None,
                    unavailable: unknown,
                },
                InputPin {
                    pin: CpuPin::TEST,
                    label: if is_286 || is_386 { "BUSY" } else { "TEST" },
                    tooltip: if is_286 || is_386 {
                        "Checked: BUSY asserted (pin LOW). Unchecked: BUSY deasserted (pin HIGH)."
                    } else {
                        "Checked: TEST HIGH (WAIT waits). Unchecked: TEST LOW (WAIT can continue)."
                    },
                    inverted: is_286 || is_386,
                    value: None,
                    unavailable: if is_386 {
                        Some("BUSY is tied HIGH on the 386 shield and cannot be controlled.")
                    } else {
                        unknown
                    },
                },
                InputPin {
                    pin: CpuPin::HOLD,
                    label: "HOLD",
                    tooltip: "Checked: HOLD asserted (pin HIGH), requesting the bus. Unchecked: HOLD deasserted (pin LOW).",
                    inverted: false,
                    value: None,
                    unavailable: if is_286 {
                        Some("Placeholder: HOLD is not connected on the 286 setup.")
                    } else if is_386 {
                        None
                    } else {
                        Some("HOLD is not mapped on this shield.")
                    },
                },
            ],
        }
    }

    pub fn apply(&mut self, readouts: Vec<PinReadout>) {
        for readout in readouts {
            if let Some(pin) = self.pins.iter_mut().find(|pin| pin.pin == readout.pin) {
                pin.read(readout.value);
            }
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui, busy: bool, events: &mut GuiEventQueue) {
        for pin in &mut self.pins {
            let mut checked = pin.value.unwrap_or(false);
            let response = ui
                .add_enabled(
                    !busy && pin.unavailable.is_none() && pin.value.is_some(),
                    egui::Checkbox::new(&mut checked, pin.label).indeterminate(pin.value.is_none()),
                )
                .on_hover_text(pin.tooltip)
                .on_disabled_hover_text(pin.unavailable.unwrap_or(if busy {
                    "Waiting for the current serial operation."
                } else {
                    "Waiting for a pin state read from the server."
                }));
            if response.changed() {
                events.push(GuiEvent::WritePin {
                    pin: pin.pin,
                    value: checked ^ pin.inverted,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arduinox86_client::ServerCommand;

    #[test]
    fn ready_is_logical_busy_is_raw() {
        for cpu in [
            ServerCpuType::Intel8088,
            ServerCpuType::Intel80286,
            ServerCpuType::Intel80386,
        ] {
            let mut controls = CpuInputControls::new(cpu);
            assert!(!controls.pins[0].inverted);
            controls.pins[0].read(Ok(true));
            assert_eq!(controls.pins[0].value, Some(true));
            controls.pins[0].read(Ok(false));
            assert_eq!(controls.pins[0].value, Some(false));
        }

        let mut controls = CpuInputControls::new(ServerCpuType::Intel80286);
        controls.pins[1].read(Ok(false)); // 286 BUSY is a raw active-low pin.
        assert_eq!(controls.pins[1].value, Some(true));
        controls.pins[1].read(Ok(true));
        assert_eq!(controls.pins[1].value, Some(false));

        let mut controls = CpuInputControls::new(ServerCpuType::Intel80386);
        assert!(controls.pins[1].unavailable.is_some()); // BUSY is hardwired.
        controls.pins[2].read(Ok(true));
        assert_eq!(controls.pins[2].value, Some(true));
    }

    #[test]
    fn unsupported_hold_requires_reconnect() {
        let mut controls = CpuInputControls::new(ServerCpuType::Intel80386);
        let hold = &mut controls.pins[2];
        hold.read(Err(CpuClientError::CommandFailed(ServerCommand::CmdReadPin)));
        assert_eq!(hold.value, None);
        assert!(hold.unavailable.is_some());
        let reconnected = CpuInputControls::new(ServerCpuType::Intel80386);
        assert!(reconnected.pins[2].unavailable.is_none());
    }

    #[test]
    fn refresh_recovers_transport_error() {
        let mut controls = CpuInputControls::new(ServerCpuType::Intel80286);
        let ready = &mut controls.pins[0];
        ready.read(Ok(true));
        ready.read(Err(CpuClientError::ReadFailure));
        assert_eq!(ready.value, None);
        assert!(ready.unavailable.is_none());
        ready.read(Ok(false));
        assert_eq!(ready.value, Some(false));
    }
}
