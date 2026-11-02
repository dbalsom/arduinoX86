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

use std::{
    default::Default,
    fs,
    io::Cursor,
    path::{Path, PathBuf},
    time::Duration,
};

use crate::{
    assembler::Assembler,
    client::ClientContext,
    config::ConfigFile,
    enums::{BinaryBlobType, ClientControlState, CpuStateType, MountAddress, ScheduleType},
    events::{FrontendThreadEvent, GuiEvent, GuiEventQueue},
    resource_manager::ResourceManager,
    scheduler::Scheduler,
    serial_manager::SerialManager,
    structs::{BinaryBlob, ScheduledEvent},
    style::custom_style,
    window_manager::WindowManager,
    windows::{ClientWindow, MemoryViewer, RegisterWindow},
};
use anyhow::{bail, Result};
use arduinox86_client::{ProgramState, RegisterSetType, RemoteCpuRegisters, ServerFlags, ServerStatus};
use clap::Parser;
use egui::{
    containers::menu::{MenuButton, MenuConfig},
    PopupCloseBehavior,
};
use egui_extras::syntax_highlighting::SyntectSettings;
use egui_notify::Toasts;
use syntect::parsing::{SyntaxSet, SyntaxSetBuilder};
use tempfile::NamedTempFile;

pub const SHORT_NOTIFICATION_TIME: Option<Duration> = Some(Duration::from_secs(2));
pub const NORMAL_NOTIFICATION_TIME: Option<Duration> = Some(Duration::from_secs(5));
pub const LONG_NOTIFICATION_TIME: Option<Duration> = Some(Duration::from_secs(8));

pub const SERVER_UPDATE_RATE: u64 = 1;
pub const SERVER_UPDATE_MS: u64 = 1000 / SERVER_UPDATE_RATE;

#[derive(Parser, Debug)]
#[command(author, version, about)]
struct Cli {
    /// Path to the TOML config file
    #[arg(long, value_name = "FILE", default_value = "./cfg/arduinox86_gui.toml")]
    config_file: PathBuf,
}

#[derive(Default)]
pub struct TransientAppState {
    ctx_init: bool,
    app_init: bool,
    config: ConfigFile,
    serial_manager: SerialManager,
    resource_manager: ResourceManager,
    last_program_state: Option<ProgramState>,
    selected_serial_port: usize,

    client_ctx: Option<ClientContext>,
    client_window: ClientWindow,
    window_manager: WindowManager,
    initial_register_window: RegisterWindow,
    final_register_window: RegisterWindow,
    memory_viewer_window: MemoryViewer,
    scheduler: Scheduler,
    event_queue: GuiEventQueue,
    error_msg: Option<String>,

    auto_refresh: bool,
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct GuiState {
    #[serde(skip)]
    toasts: Toasts,
    pub(crate) syntax_set: SyntaxSet,
    #[serde(skip)]
    pub(crate) syntect_settings: SyntectSettings,
}

impl Default for GuiState {
    fn default() -> Self {
        Self {
            toasts: Toasts::new().with_anchor(egui_notify::Anchor::BottomRight),
            syntax_set: SyntaxSet::load_defaults_newlines(),
            syntect_settings: SyntectSettings::default(),
        }
    }
}

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct App {
    pub(crate) gs: GuiState,
    #[serde(skip)]
    ts: TransientAppState,

    #[serde(skip)]
    pub(crate) thread_sender:   crossbeam_channel::Sender<FrontendThreadEvent>,
    #[serde(skip)]
    pub(crate) thread_receiver: crossbeam_channel::Receiver<FrontendThreadEvent>,
}

impl Default for App {
    fn default() -> Self {
        let (thread_sender, thread_receiver) = crossbeam_channel::unbounded();

        let new_app = Self {
            gs: Default::default(),
            ts: TransientAppState {
                ..TransientAppState::default()
            },
            thread_sender,
            thread_receiver,
        };

        let mut syntaxes_found = 0;
        for syntax in new_app.gs.syntax_set.syntaxes() {
            log::debug!("Have App::default() syntaxes: {}", syntax.name);
            syntaxes_found += 1;
        }

        log::debug!("Found {} show syntaxes in App::default()", syntaxes_found);

        new_app
    }
}

impl App {
    /// Initialize the egui context, for visuals, etc.
    /// Tried doing this in new() but it didn't take effect.
    pub fn ctx_init(&mut self, ctx: &egui::Context) {
        ctx.set_visuals(egui::Visuals::dark());

        let mut custom_style = custom_style();

        // Make header smaller.
        use egui::{FontFamily::Proportional, FontId, TextStyle::*};

        custom_style.text_styles.entry(Heading).and_modify(|text_style| {
            *text_style = FontId::new(14.0, Proportional);
        });
        custom_style.spacing.window_margin = egui::Margin::from(4);

        let mut fonts = egui::FontDefinitions::default();
        egui_phosphor::add_to_fonts(&mut fonts, egui_phosphor::Variant::Regular);
        ctx.set_fonts(fonts);

        ctx.set_style(custom_style);

        self.ts.ctx_init = true;
    }

    pub fn app_init(&mut self) {
        self.ts.serial_manager.refresh();
        self.ts.selected_serial_port = self.ts.serial_manager.preferred_port_index();
        let event = ScheduledEvent::new(ScheduleType::Repeat, GuiEvent::PollStatus, SERVER_UPDATE_MS);
        self.ts.scheduler.add_event(event);
        self.ts.app_init = true;
    }

    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        let cli = Cli::parse();

        // Load the config file.
        let config_text = match fs::read_to_string(&cli.config_file) {
            Ok(text) => text,
            Err(e) => {
                log::error!("Failed to read config file {}: {}", cli.config_file.display(), e);
                // exit
                std::process::exit(1);
            }
        };

        let config: ConfigFile = match toml::from_str(&config_text) {
            Ok(cfg) => cfg,
            Err(e) => {
                log::error!("Failed to parse config file {}: {}", cli.config_file.display(), e);
                // exit
                std::process::exit(1);
            }
        };

        // Create directories if they don't exist.
        if let Err(e) = fs::create_dir_all(&config.assembly_output_path) {
            log::error!(
                "Failed to create data directory {}: {}",
                config.assembly_output_path.display(),
                e
            );
            // exit
            std::process::exit(1);
        }

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            let mut restored_app: App = eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default();
            // Recreate SyntectSettings
            restored_app.gs.syntect_settings = SyntectSettings {
                ps: restored_app.gs.syntax_set.clone(),
                ts: syntect::highlighting::ThemeSet::load_defaults(),
            };

            return restored_app;
        }

        let mut ss_builder = SyntaxSetBuilder::new();

        ss_builder
            .add_from_folder("./syntax", true)
            .expect("failed to load syntax definitions");

        for syntax in ss_builder.syntaxes() {
            log::debug!("Loaded syntax: {}", syntax.name);
        }

        let syntax_set = ss_builder.build();

        let mut syntaxes_found = 0;
        for syntax in syntax_set.syntaxes() {
            log::debug!("Have original syntax: {}", syntax.name);
            syntaxes_found += 1;
        }
        log::debug!("Found {} original syntaxes in GuiState::SyntaxSet", syntaxes_found);

        let new_app = App {
            gs: GuiState {
                toasts: Toasts::new().with_anchor(egui_notify::Anchor::BottomRight),
                syntax_set: syntax_set.clone(),
                syntect_settings: SyntectSettings {
                    ps: syntax_set,
                    ts: syntect::highlighting::ThemeSet::load_defaults(),
                },
            },
            ts: TransientAppState {
                config,
                serial_manager: SerialManager::new(),
                ..Default::default()
            },
            ..Default::default()
        };

        let mut syntaxes_found = 0;
        for syntax in new_app.gs.syntax_set.syntaxes() {
            log::debug!("Have App::new() syntaxes: {}", syntax.name);
            syntaxes_found += 1;
        }

        log::warn!("Found {} show syntaxes in App::new()", syntaxes_found);

        new_app
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if !self.ts.ctx_init {
            self.ctx_init(ctx);
        }
        if !self.ts.app_init {
            self.app_init();
        }

        self.gs.toasts.show(ctx);

        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if let Some(c_ctx) = &mut self.ts.client_ctx {
                        if c_ctx.control_state() == ClientControlState::Setup {
                            ui.horizontal(|ui| {
                                if ui.button("New Assembly Listing").clicked() {
                                    self.ts.window_manager.new_code_window("Program", None);
                                    log::debug!("New assembly listing created.");
                                }
                            });

                            ui.horizontal(|ui| {
                                if ui.button("Load Assembly Listing...").clicked() {
                                    if let Some(path) = rfd::FileDialog::new()
                                        .add_filter("Assembly Files", &["asm", "nasm"])
                                        .pick_file()
                                    {
                                        log::debug!("Loading Assembly file: {}", path.display());

                                        match self.load_asm(path) {
                                            Ok(()) => {
                                                self.gs
                                                    .toasts
                                                    .success("Assembly file loaded successfully!")
                                                    .duration(NORMAL_NOTIFICATION_TIME);
                                                log::debug!("Assembly file loaded successfully.");
                                            }
                                            Err(e) => {
                                                log::error!("Failed to load assembly file: {}", e);
                                                self.ts.error_msg =
                                                    Some(format!("Failed to load assembly file: {}", e));
                                                self.gs
                                                    .toasts
                                                    .error(format!("Failed to load assembly file: {}", e))
                                                    .duration(LONG_NOTIFICATION_TIME);
                                            }
                                        }
                                    }
                                }
                            });

                            ui.separator();

                            ui.horizontal(|ui| {
                                if ui.button("Load Binary...").clicked() {
                                    if let Some(path) = rfd::FileDialog::new()
                                        .add_filter("Binary Files", &["bin", "hex", "rom"])
                                        .pick_file()
                                    {
                                        match BinaryBlob::from_path(
                                            &*path
                                                .file_name()
                                                .map(|f| f.to_string_lossy().to_string())
                                                .unwrap_or_else(|| "Program".to_string()),
                                            MountAddress::CsIp,
                                            BinaryBlobType::Program,
                                            &path,
                                        ) {
                                            Ok(blob) => match self.ts.resource_manager.add_blob(blob) {
                                                Ok(binary_view) => {
                                                    self.ts
                                                        .window_manager
                                                        .add_blob(binary_view.name().to_string(), binary_view);
                                                    self.gs
                                                        .toasts
                                                        .info("Program binary loaded successfully!")
                                                        .duration(NORMAL_NOTIFICATION_TIME);
                                                    log::info!("Program binary loaded successfully.");
                                                }
                                                Err(e) => {
                                                    log::error!("Failed to add program binary: {}", e);
                                                    self.gs
                                                        .toasts
                                                        .error(format!("Failed to add program binary: {}", e))
                                                        .duration(LONG_NOTIFICATION_TIME);
                                                    self.ts.error_msg =
                                                        Some(format!("Failed to add program binary: {}", e));
                                                }
                                            },
                                            Err(e) => {
                                                log::error!("Failed to create program blob: {}", e);
                                                self.ts.error_msg = Some(format!("Failed to load program: {}", e));
                                            }
                                        }
                                    }
                                }
                            });
                        }
                    }

                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });

                MenuButton::new("Serial Port")
                    .config(MenuConfig::default().close_behavior(PopupCloseBehavior::CloseOnClickOutside))
                    .ui(ui, |ui| {
                        if ui.button("Refresh").clicked() {
                            let selected_name = self.ts.serial_manager.ports()
                                .get(self.ts.selected_serial_port)
                                .map(|port| port.port_name.clone());
                            self.ts.serial_manager.refresh();
                            self.ts.selected_serial_port = self.ts.serial_manager.ports()
                                .iter()
                                .position(|port| Some(&port.port_name) == selected_name.as_ref())
                                .unwrap_or_else(|| self.ts.serial_manager.preferred_port_index());
                        }
                        ui.separator();
                        for (i, port) in self.ts.serial_manager.port_display_names().iter().enumerate() {
                            ui.horizontal(|ui| {
                                if ui.radio_value(&mut self.ts.selected_serial_port, i, port).clicked() {
                                    log::debug!("Selected port: {}", port);
                                }
                            });
                        }
                    });
                ui.add_space(16.0);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            // The central panel the region left after adding TopPanel's and SidePanel's
            ui.heading("ArduinoX86 GUI ");
            ui.separator();

            let have_port = self
                .ts
                .serial_manager
                .port_names()
                .get(self.ts.selected_serial_port)
                .is_some();

            if have_port && self.ts.client_ctx.is_none() {
                if ui
                    .button(
                        egui::RichText::new(format!(
                            "⮉ Connect to ArduinoX86 Server on {}",
                            self.ts
                                .serial_manager
                                .port_names()
                                .get(self.ts.selected_serial_port)
                                .unwrap_or(&"No Port Selected".to_string())
                        ))
                        .size(18.0),
                    )
                    .clicked()
                {
                    // Do clicky stuff
                    match ClientContext::new(self.ts.selected_serial_port, &mut self.ts.serial_manager) {
                        Ok(client_ctx) => {
                            self.ts.error_msg = None;
                            self.ts.client_window.init(&client_ctx);
                            self.ts.initial_register_window.set_regs(&client_ctx.initial_state().regs, None);
                            *self.ts.initial_register_window.open_mut() = true;
                            *self.ts.final_register_window.open_mut() = false;
                            self.ts.last_program_state = None;
                            self.ts.client_ctx = Some(client_ctx);

                            log::debug!(
                                "Connected to ArduinoX86 server on port: {}",
                                self.ts.selected_serial_port
                            );
                        }
                        Err(e) => {
                            log::error!("Failed to connect to ArduinoX86 server: {}", e);
                            self.ts.client_ctx = None;
                        }
                    }
                }
            }
            else if self.ts.client_ctx.is_some() {
                ui.label("Connected!");
            }
            else {
                ui.label("Select a serial port to connect.");
            }

            if let Some(err_msg) = &self.ts.error_msg {
                ui.colored_label(egui::Color32::RED, format!("Error: {}", err_msg));
            }

            ui.with_layout(egui::Layout::bottom_up(egui::Align::LEFT), |ui| {
                egui::warn_if_debug_build(ui);
            });
        });

        // Render floating windows.
        if let Some(client_ctx) = &mut self.ts.client_ctx {
            self.ts
                .client_window
                .show(ctx, client_ctx, &mut self.ts.event_queue, &mut self.gs.toasts);
            let mut update_state = false;
            // Get the initial state from the client.
            let mut initial_state = client_ctx.initial_state_mut();
            let mut new_state = initial_state.clone();

            if !self.ts.initial_register_window.open() {
                self.ts.initial_register_window.set_regs(&initial_state.regs, None);
                *self.ts.initial_register_window.open_mut() = true;
            }

            match &initial_state.regs {
                RemoteCpuRegisters::V2(_) | RemoteCpuRegisters::V3(_) => {
                    self.ts.initial_register_window.show(
                        ctx,
                        CpuStateType::Initial,
                        &mut self.ts.event_queue,
                    );

                    let new_regs = self.ts.initial_register_window.regs();

                    new_state.regs = new_regs;
                    update_state = true;
                }
                _ => {
                    log::warn!(
                        "Unsupported register type: {}",
                        RegisterSetType::from(&initial_state.regs)
                    );
                    self.ts.error_msg = Some("Unsupported register type".to_string());
                }
            }

            self.ts.final_register_window.show(
                ctx,
                CpuStateType::Final,
                &mut self.ts.event_queue,
            );

            //self.ts.code_editor_window.show(ctx, &mut self.ts.event_queue);
            self.ts.window_manager.show(
                ctx,
                client_ctx,
                &mut self.ts.resource_manager,
                &self.gs.syntect_settings,
                &mut self.ts.event_queue,
            );
            self.ts
                .memory_viewer_window
                .show(ctx, client_ctx, &mut self.ts.event_queue);

            self.ts.scheduler.run(&mut self.ts.event_queue);

            if update_state {
                client_ctx.set_initial_state(&new_state);
            }
        }

        // Handle events.
        self.handle_events(ctx);

        ctx.request_repaint();
    }

    /// Called by the framework to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }
}

impl App {
    /// Handle events from the GUI event queue.
    fn handle_events(&mut self, _c_ctx: &egui::Context) {
        let mut new_events = Vec::new();
        if let Some(client_ctx) = &mut self.ts.client_ctx {
            while let Some(event) = self.ts.event_queue.pop() {
                match event {
                    GuiEvent::ResetState => {
                        self.ts.last_program_state = None;
                        *self.ts.final_register_window.open_mut() = false;
                    }
                    GuiEvent::LoadRegisters => {
                        let program_state = client_ctx.program_state();

                        // Clear automatic execution flag
                        match client_ctx.set_flag_state(ServerFlags::EXECUTE_AUTOMATIC, false) {
                            Ok(_) => {
                                self.gs
                                    .toasts
                                    .success("Automatic execution flag cleared.")
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!("Automatic execution flag cleared.");
                            }
                            Err(e) => {
                                self.gs
                                    .toasts
                                    .error(format!("Failed to clear automatic execution flag: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);
                                log::error!("Failed to clear automatic execution flag: {}", e);
                            }
                        }

                        let load_regs = self.ts.initial_register_window.load_regs();

                        let mut buf = Cursor::new(Vec::<u8>::new());

                        let mut regs = &load_regs;
                        let regs_b;

                        if let Ok(ProgramState::StoreDoneSmm) = program_state {
                            // Try to convert regs to V3B for SMM loading.
                            log::debug!("Converting registers to V3B for SMM loading.");
                            if let Some(regs_converted) = regs.to_b() {
                                regs_b = regs_converted;
                                regs = &regs_b;
                            }
                            else {
                                log::error!("Failed to convert registers to V3B for SMM loading!");
                            }
                        }

                        regs.write(&mut buf).unwrap_or_else(|e| {
                            log::error!("Failed to write registers: {}", e);
                        });

                        log::debug!("Wrote {} bytes of register data to buffer.", buf.get_ref().len());

                        match client_ctx
                            .client
                            .load_registers_from_buf(RegisterSetType::from(regs), buf.get_ref())
                        {
                            Ok(_) => {
                                log::debug!("Registers loaded successfully.");
                                self.gs
                                    .toasts
                                    .success("Registers loaded successfully!")
                                    .duration(NORMAL_NOTIFICATION_TIME);
                            }
                            Err(e) => {
                                log::error!("Failed to load registers: {}", e);
                                self.gs
                                    .toasts
                                    .error(format!("Failed to load registers: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);
                                self.ts.error_msg = Some(format!("Failed to load registers: {}", e));
                            }
                        }

                        match self.ts.client_window.push_cycle(client_ctx, false) {
                            Ok(_) => {
                                log::debug!("Pushed CPU cycle successfully.");
                            }
                            Err(e) => {
                                log::error!("Failed to push CPU cycle: {}", e);
                                self.gs
                                    .toasts
                                    .error(format!("Failed to push CPU cycle: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);
                                self.ts.error_msg = Some(format!("Failed to push CPU cycle: {}", e));
                            }
                        }
                    }
                    GuiEvent::EraseMemory => match client_ctx.erase_memory() {
                        Ok(_) => {
                            log::debug!("Memory erased successfully.");
                            self.gs
                                .toasts
                                .success("Memory erased successfully!")
                                .duration(NORMAL_NOTIFICATION_TIME);
                        }
                        Err(e) => {
                            log::error!("Failed to erase memory: {}", e);
                            self.gs
                                .toasts
                                .error(format!("Failed to erase memory: {}", e))
                                .duration(LONG_NOTIFICATION_TIME);
                            self.ts.error_msg = Some(format!("Failed to erase memory: {}", e));
                        }
                    },
                    GuiEvent::ReadMemory { address, size } => match client_ctx.read_memory(address, size) {
                        Ok(data) => {
                            self.ts.memory_viewer_window.set_data(data);
                            log::debug!(
                                "Memory read successfully from address {:#x} with size {}",
                                address,
                                size
                            );
                        }
                        Err(e) => {
                            log::error!("Failed to read memory: {}", e);
                            self.ts.error_msg = Some(format!("Failed to read memory: {}", e));
                        }
                    },
                    GuiEvent::UploadBlob {
                        blob_name,
                        mount_address,
                        size,
                    } => {
                        if let Some(blob) = self.ts.resource_manager.blob(&blob_name) {
                            let resolved_mount_address = match mount_address {
                                MountAddress::FlatAddress(addr) => addr,
                                MountAddress::CsIp => self
                                    .ts
                                    .initial_register_window
                                    .code_address(),
                            };

                            log::debug!(
                                "Loading binary blob: {} at address {:08x}",
                                blob_name,
                                resolved_mount_address
                            );

                            let slice_size = std::cmp::min(size.unwrap_or(blob.data.len()), blob.data.len());
                            if let Err(e) = client_ctx
                                .client
                                .set_memory(resolved_mount_address, &blob.data[0..slice_size])
                            {
                                self.gs
                                    .toasts
                                    .error(format!("Failed to load binary blob: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);

                                log::error!("Failed to load binary blob: {}", e);
                                self.ts.error_msg = Some(format!("Failed to load binary blob: {}", e));
                                return;
                            }
                            else {
                                self.gs
                                    .toasts
                                    .success(format!("Binary blob: {} loaded successfully!", blob.name))
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!(
                                    "Binary blob: {} loaded successfully at address {:#x}",
                                    blob.name,
                                    resolved_mount_address
                                );
                            }
                        }
                        else {
                            log::error!("Blob {} not found for upload.", blob_name);
                            self.gs
                                .toasts
                                .error(format!("Blob {} not found for upload.", blob_name))
                                .duration(LONG_NOTIFICATION_TIME);
                            self.ts.error_msg = Some(format!("Blob {} not found for upload.", blob_name));
                        }
                    }
                    GuiEvent::RunProgram => {
                        // Load the binary resources into memory.
                        for blob in self.ts.resource_manager.blobs() {
                            let resolved_mount_address = match blob.mount_address {
                                MountAddress::FlatAddress(addr) => addr,
                                MountAddress::CsIp => self
                                    .ts
                                    .initial_register_window
                                    .code_address(),
                            };

                            log::debug!(
                                "Loading binary blob: {} at address {:08x}",
                                blob.name,
                                resolved_mount_address
                            );

                            if let Err(e) = client_ctx.client.set_memory(resolved_mount_address, &blob.data) {
                                self.gs
                                    .toasts
                                    .error(format!("Failed to load binary blob: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);

                                log::error!("Failed to load binary blob: {}", e);
                                self.ts.error_msg = Some(format!("Failed to load binary blob: {}", e));
                                return;
                            }
                            else {
                                self.gs
                                    .toasts
                                    .success(format!("Binary blob: {} loaded successfully!", blob.name))
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!(
                                    "Binary blob: {} loaded successfully at address {:#x}",
                                    blob.name,
                                    resolved_mount_address
                                );
                            }
                        }

                        match client_ctx.set_flag_state(ServerFlags::EXECUTE_AUTOMATIC, true) {
                            Ok(_) => {
                                self.gs
                                    .toasts
                                    .success("Automatic execution flag set.")
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!("Automatic execution flag set.");
                            }
                            Err(e) => {
                                self.gs
                                    .toasts
                                    .error(format!("Failed to set automatic execution flag: {}", e))
                                    .duration(LONG_NOTIFICATION_TIME);
                                log::error!("Failed to set automatic execution flag: {}", e);
                            }
                        }

                        // Load registers.
                        let load_regs = self.ts.initial_register_window.load_regs();

                        let mut buf = Cursor::new(Vec::<u8>::new());

                        load_regs
                            .write(&mut buf)
                            .expect("Failed to write registers to buffer");

                        if let Err(e) = client_ctx
                            .client
                            .load_registers_from_buf(RegisterSetType::from(&load_regs), buf.get_ref())
                        {
                            self.gs
                                .toasts
                                .error(format!("Failed to load registers: {}", e))
                                .duration(LONG_NOTIFICATION_TIME);
                            log::error!("Failed to load registers: {}", e);
                            self.ts.error_msg = Some(format!("Failed to load registers: {}", e));
                        }
                        else {
                            self.gs
                                .toasts
                                .success("Registers loaded successfully!")
                                .duration(NORMAL_NOTIFICATION_TIME);
                            log::debug!("Registers loaded successfully.");
                        }
                    }
                    GuiEvent::AssembleProgram { program_name } => {
                        let mut new_blob = None;
                        let mut update_blob = None;
                        if let Some(code_editor) = self.ts.window_manager.code_window_mut(&program_name) {
                            let mut assembler = Assembler::default();

                            match NamedTempFile::with_suffix(".bin") {
                                Ok(file) => match assembler.assemble_str(code_editor.code(), &file.path()) {
                                    Ok(binary) => {
                                        log::debug!("Assembly successful for code editor: {}", program_name);

                                        if self.ts.resource_manager.blob_exists(&program_name) {
                                            log::warn!("Blob with name {} already exists, overwriting.", program_name);
                                            if let Ok(()) = self.ts.resource_manager.update_blob(&program_name, &binary)
                                            {
                                                update_blob = Some(binary);
                                            }
                                        }
                                        else {
                                            new_blob = Some(binary);
                                        }

                                        self.gs
                                            .toasts
                                            .success(format!("Assembly successful for {}!", program_name))
                                            .duration(NORMAL_NOTIFICATION_TIME);
                                        code_editor.set_assembler_output(assembler.stdout());
                                    }
                                    Err(e) => {
                                        let stderr = assembler.stderr();
                                        log::error!(
                                            "Assembly failed for code editor {}: {}: {}",
                                            program_name,
                                            e,
                                            stderr
                                        );
                                        self.gs
                                            .toasts
                                            .error(format!("Assembly failed: {}", e))
                                            .duration(LONG_NOTIFICATION_TIME);

                                        code_editor.set_assembler_output(assembler.stderr());
                                    }
                                },
                                Err(e) => {
                                    log::error!("Failed to create temporary file for assembly: {}", e);
                                    self.ts.error_msg = Some(format!("Failed to create temporary file: {}", e));
                                    continue;
                                }
                            }
                        }

                        if let Some(binary) = new_blob {
                            match self.ts.resource_manager.add_blob(BinaryBlob::new(
                                program_name.clone(),
                                MountAddress::CsIp,
                                BinaryBlobType::Program,
                                binary,
                            )) {
                                Ok(binary_view) => {
                                    log::debug!("Binary blob {} added successfully", program_name);
                                    self.ts.window_manager.add_blob(program_name.clone(), binary_view);
                                }
                                Err(e) => {
                                    log::error!("Failed to add binary blob {}: {}", program_name, e);
                                    self.gs
                                        .toasts
                                        .error(format!("Failed to add binary blob {}: {}", program_name, e))
                                        .duration(LONG_NOTIFICATION_TIME);
                                }
                            }
                        }

                        if let Some(binary) = update_blob {
                            if let Some(window) = self.ts.window_manager.blob_window_mut(&program_name) {
                                window.set_data(&binary);
                            }
                        }
                    }
                    GuiEvent::PollStatus => {
                        // Get the server status. This event is scheduled automatically.
                        match client_ctx.client.server_status() {
                            Ok(status) => {
                                //log::debug!("Server status: {:?}", status);
                                self.ts.client_window.set_server_status(client_ctx, status);

                                if self.ts.last_program_state.is_none()
                                    || (Some(status.state) != self.ts.last_program_state)
                                {
                                    log::info!("Program state changed: {:?}", status.state);

                                    match status.state {
                                        ProgramState::StoreDone | ProgramState::StoreDoneSmm => {
                                            // Get the register file.

                                            match client_ctx.client.store_registers() {
                                                Ok(final_regs) => {
                                                    let state = client_ctx.initial_state();

                                                    self.ts
                                                        .final_register_window
                                                        .set_regs(&state.regs, Some(&final_regs));
                                                    *self.ts.final_register_window.open_mut() = true;
                                                    log::debug!(
                                                        "Registers updated after program completion: {:?}",
                                                        final_regs
                                                    );
                                                }
                                                Err(e) => {
                                                    log::error!(
                                                        "Failed to get registers after program completion: {}",
                                                        e
                                                    );
                                                    self.gs
                                                        .toasts
                                                        .error(format!("Failed to get final registers: {}", e))
                                                        .duration(LONG_NOTIFICATION_TIME);
                                                    self.ts.error_msg =
                                                        Some(format!("Failed to get final registers: {}", e));
                                                }
                                            }
                                        }
                                        _ => {
                                            // Do other states
                                        }
                                    }

                                    self.ts.last_program_state = Some(status.state);
                                }
                            }
                            Err(e) => {
                                log::error!("Failed to get server status: {}", e);
                                self.ts.error_msg = Some(format!("Failed to get server status: {}", e));
                            }
                        }
                    }
                    GuiEvent::ClearCycleLog => match client_ctx.client.clear_cycle_log() {
                        Ok(_) => {
                            log::debug!("Cycle log cleared successfully.");
                            self.gs
                                .toasts
                                .success("Cycle log cleared successfully!")
                                .duration(NORMAL_NOTIFICATION_TIME);
                        }
                        Err(e) => {
                            log::error!("Failed to clear cycle log: {}", e);
                            self.gs
                                .toasts
                                .error(format!("Failed to clear cycle log: {}", e))
                                .duration(LONG_NOTIFICATION_TIME);
                            self.ts.error_msg = Some(format!("Failed to clear cycle log: {}", e));
                        }
                    },
                    GuiEvent::ToggleRefreshMemory {
                        enabled: state,
                        hertz: rate,
                    } => {
                        if state != self.ts.auto_refresh {
                            if state {
                                let new_event = ScheduledEvent::new(
                                    ScheduleType::Repeat,
                                    GuiEvent::RefreshMemory,
                                    1000 / rate as u64,
                                );
                                self.ts.scheduler.add_event(new_event);
                                self.gs
                                    .toasts
                                    .info("Auto-refresh enabled.")
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!("Auto-refresh enabled.");
                            }
                            else {
                                self.ts.scheduler.remove_event_type(&GuiEvent::RefreshMemory);
                                self.gs
                                    .toasts
                                    .info("Auto-refresh disabled.")
                                    .duration(NORMAL_NOTIFICATION_TIME);
                                log::debug!("Auto-refresh disabled.");
                            }
                            self.ts.auto_refresh = state;
                        }
                    }
                    GuiEvent::RefreshMemory => {
                        log::debug!("Refreshing memory view.");
                        let new_event = self.ts.memory_viewer_window.make_refresh_event();
                        new_events.push(new_event);
                    }
                }
            }
        }

        // Add any events generated by processed events
        for event in new_events {
            self.ts.event_queue.push(event);
        }
    }

    fn load_asm(&mut self, path: impl AsRef<Path>) -> Result<()> {
        match fs::read_to_string(&path) {
            Ok(code) => {
                self.ts.window_manager.new_code_window(
                    path.as_ref()
                        .file_name()
                        .map_or("Program", |f| f.to_str().unwrap_or("Program")),
                    Some(code),
                );

                Ok(())
            }
            Err(e) => bail!("Failed to read assembly file {}: {}", path.as_ref().display(), e),
        }
    }
}
