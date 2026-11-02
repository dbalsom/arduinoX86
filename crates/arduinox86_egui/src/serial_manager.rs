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
use serialport::{SerialPortInfo, SerialPortType};

#[derive(Default)]
pub struct SerialManager {
    ports: Vec<SerialPortInfo>,
}

impl SerialManager {
    pub fn new() -> Self {
        SerialManager {
            ports: Self::enumerate_and_filter_ports(),
        }
    }

    pub fn refresh(&mut self) {
        self.ports = Self::enumerate_and_filter_ports();
    }

    // Don't ignore 'unknown' ports, but prefer others first
    pub fn preferred_port_index(&self) -> usize {
        self.ports
            .iter()
            .position(|port| !matches!(port.port_type, SerialPortType::Unknown))
            .unwrap_or(0)
    }

    fn enumerate_and_filter_ports() -> Vec<SerialPortInfo> {
        serialport::available_ports()
            .unwrap_or_else(|_| vec![])
            .into_iter()
            .filter_map(|p| {
                // Filter port names containing "USB-to-Serial"
                if let SerialPortType::UsbPort(info) = &p.port_type {
                    if let Some(product) = &info.product {
                        if product.contains("USB-to-Serial") {
                            return None;
                        }
                    }
                }

                Some(p)
            })
            .collect()
    }

    pub fn port_display_names(&self) -> Vec<String> {
        self.ports
            .iter()
            .map(|port| match &port.port_type {
                SerialPortType::UsbPort(info) => {
                    format!(
                        "{} ({})",
                        port.port_name,
                        info.product.as_deref().unwrap_or("Unknown USB Device")
                    )
                }
                SerialPortType::BluetoothPort => {
                    format!("{} (Bluetooth)", port.port_name)
                }
                SerialPortType::PciPort => {
                    format!("{} (PCI)", port.port_name)
                }
                SerialPortType::Unknown => {
                    format!("{} (Unknown)", port.port_name)
                }
            })
            .collect()
    }

    pub fn port_names(&self) -> Vec<String> {
        self.ports.iter().map(|port| port.port_name.clone()).collect()
    }

    pub fn ports(&self) -> &[SerialPortInfo] {
        &self.ports
    }
}
