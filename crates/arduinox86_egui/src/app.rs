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
    path::{Path, PathBuf},
    time::Duration,
};

use crate::{
    assembler::Assembler,
    client::ClientContext,
    config::ConfigFile,
    enums::{BinaryBlobType, ClientControlState, CpuStateType, MountAddress, ScheduleType},
    events::{FrontendThreadEvent, GuiEvent, GuiEventQueue},
    memory_upload::MemoryUpload,
    resource_manager::ResourceManager,
    scheduler::Scheduler,
    serial_manager::SerialManager,
    serial_worker::{SerialCommand, SerialOutcome, SerialUpdate, SerialWorker},
    structs::{BinaryBlob, ScheduledEvent},
    style::custom_style,
    window_manager::WindowManager,
    windows::{ClientWindow, DisassemblyWindow, MemoryViewer, RegisterWindow},
};
use anyhow::{bail, Result};
use arduinox86_client::{RegisterSetType, RemoteCpuRegisters, ServerFlags};
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
    serial_worker: Option<SerialWorker>,
    selected_serial_port: usize,

    client_ctx: Option<ClientContext>,
    client_window: ClientWindow,
    window_manager: WindowManager,
    initial_register_window: RegisterWindow,
    final_register_window: RegisterWindow,
    memory_viewer_window: MemoryViewer,
    disassembly_window: DisassemblyWindow,
    scheduler: Scheduler,
    event_queue: GuiEventQueue,
    error_msg: Option<String>,

    auto_refresh: bool,
    poll_requested: bool,
    memory_refresh_requested: bool,
    next_background_memory: bool,
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

    pub fn app_init(&mut self, ctx: &egui::Context) {
        self.ts.serial_worker = Some(SerialWorker::new(ctx.clone()));
        self.submit_serial(SerialCommand::ListPorts);
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
            self.app_init(ctx);
        }

        self.handle_serial_replies();
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
                        if ui
                            .add_enabled(!self.serial_controls_busy(), egui::Button::new("Refresh"))
                            .clicked()
                        {
                            self.submit_serial(SerialCommand::ListPorts);
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

        crate::widgets::serial_status_bar::show(
            ctx,
            self.ts.client_ctx.as_ref(),
            self.ts.serial_worker.as_ref().and_then(SerialWorker::activity),
        );

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
                let port = self.ts.serial_manager.ports()[self.ts.selected_serial_port]
                    .port_name
                    .clone();
                if ui
                    .add_enabled(
                        !self.serial_controls_busy(),
                        egui::Button::new(
                            egui::RichText::new(format!("Connect to ArduinoX86 Server on {port}")).size(18.0),
                        ),
                    )
                    .clicked()
                {
                    self.submit_serial(SerialCommand::Connect(port));
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

        // Render cached state; no window performs serial I/O.
        let serial_busy = self.serial_controls_busy();
        if let Some(client_ctx) = &mut self.ts.client_ctx {
            client_ctx.busy = serial_busy;
            self.ts
                .client_window
                .show(ctx, client_ctx, &mut self.ts.event_queue, serial_busy);
            let mut update_state = false;
            let trap_enabled = client_ctx.cached_flags() & ServerFlags::ENABLE_TRAP_EXECUTION != 0;
            // Get the initial state from the client.
            let mut initial_state = client_ctx.initial_state_mut();
            let mut new_state = initial_state.clone();

            if !self.ts.initial_register_window.open() {
                self.ts.initial_register_window.set_regs(&initial_state.regs, None);
                *self.ts.initial_register_window.open_mut() = true;
            }

            self.ts.initial_register_window.set_trap_enabled(trap_enabled);
            match &initial_state.regs {
                RemoteCpuRegisters::V2(_) | RemoteCpuRegisters::V3(_) => {
                    self.ts
                        .initial_register_window
                        .show(ctx, CpuStateType::Initial, &mut self.ts.event_queue, serial_busy);

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

            let readout_state_type = self.ts.final_register_window.readout_state_type();
            self.ts
                .final_register_window
                .show(ctx, readout_state_type, &mut self.ts.event_queue, serial_busy);

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
            self.ts.disassembly_window.show(ctx);

            self.ts.scheduler.run(&mut self.ts.event_queue);

            if update_state {
                client_ctx.set_initial_state(&new_state);
            }
        }

        // Handle events.
        self.handle_events(ctx);
        self.submit_background();

        ctx.request_repaint();
    }

    /// Called by the framework to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }
}

impl App {
    fn serial_busy(&self) -> bool {
        self.ts.serial_worker.as_ref().map_or(true, SerialWorker::busy)
    }

    fn serial_controls_busy(&self) -> bool {
        self.ts.serial_worker.as_ref().map_or(true, SerialWorker::controls_busy)
    }

    fn submit_serial(&mut self, command: SerialCommand) -> bool {
        let result = self
            .ts
            .serial_worker
            .as_mut()
            .ok_or_else(|| anyhow::anyhow!("Serial worker has not started."))
            .and_then(|worker| worker.submit(command));
        if let Err(error) = result {
            self.gs.toasts.error(error.to_string()).duration(LONG_NOTIFICATION_TIME);
            return false;
        }
        true
    }

    fn handle_serial_replies(&mut self) {
        while let Some(reply) = self.ts.serial_worker.as_mut().and_then(SerialWorker::try_recv) {
            match reply.result {
                Ok(SerialOutcome::Ports(ports)) => {
                    let selected = self
                        .ts
                        .serial_manager
                        .ports()
                        .get(self.ts.selected_serial_port)
                        .map(|p| p.port_name.clone());
                    self.ts.serial_manager.set_ports(ports);
                    self.ts.selected_serial_port = self
                        .ts
                        .serial_manager
                        .ports()
                        .iter()
                        .position(|p| Some(&p.port_name) == selected.as_ref())
                        .unwrap_or_else(|| self.ts.serial_manager.preferred_port_index());
                }
                Ok(SerialOutcome::Connected(context, update)) => {
                    self.ts.error_msg = None;
                    self.ts.client_window.init(&context);
                    self.ts.disassembly_window.connect(context.cpu_type);
                    self.ts
                        .initial_register_window
                        .set_regs(&context.initial_state().regs, None);
                    *self.ts.initial_register_window.open_mut() = true;
                    self.ts.final_register_window.clear_readout();
                    self.ts.client_ctx = Some(context);
                    self.apply_serial_update(update);
                }
                Ok(SerialOutcome::Disconnected) => {
                    self.ts.client_ctx = None;
                    self.ts.client_window.reset_state();
                    self.ts.disassembly_window.clear();
                    *self.ts.initial_register_window.open_mut() = false;
                    self.ts.final_register_window.clear_readout();
                    self.ts.error_msg = None;
                    self.ts.event_queue.events.clear();
                    self.ts.poll_requested = false;
                    self.ts.memory_refresh_requested = false;
                }
                Ok(SerialOutcome::Update(update)) => self.apply_serial_update(update),
                Err(error) => {
                    if reply.operation == "Reading disassembly" {
                        self.ts.disassembly_window.read_failed(error.clone());
                    }
                    let message = format!("{} failed: {}", reply.operation, error);
                    log::error!("{}", message);
                    if self.ts.error_msg.as_ref() != Some(&message) {
                        self.gs.toasts.error(&message).duration(LONG_NOTIFICATION_TIME);
                    }
                    self.ts.error_msg = Some(message);
                }
            }
        }
    }

    fn apply_serial_update(&mut self, mut update: SerialUpdate) {
        if let Some(context) = &mut self.ts.client_ctx {
            if let Some(flags) = update.flags {
                if context.server_flags & ServerFlags::ENABLE_TRAP_EXECUTION != 0
                    && flags & ServerFlags::ENABLE_TRAP_EXECUTION == 0 {
                    self.ts.disassembly_window.clear();
                }
                context.server_flags = flags;
            }
            if let Some(status) = update.status {
                context.program_state = status.state;
            }
            if let Some(regs) = &update.loaded_regs {
                self.ts.disassembly_window.loaded(regs);
                self.ts.final_register_window.loaded(regs, context.server_flags & ServerFlags::ENABLE_TRAP_EXECUTION != 0);
            }
            if let Some(instruction) = &update.instruction {
                self.ts.disassembly_window.observe(instruction);
                self.ts.final_register_window.observe_instruction(instruction);
            }
            if update.memory_changed {
                self.ts.disassembly_window.refresh();
            }
            if let Some(readout) = update.disassembly.take() {
                self.ts.disassembly_window.apply(readout);
            }
            if let Some((initial, regs)) = &update.registers {
                self.ts.final_register_window.set_readout(initial, regs, update.instruction.as_ref());
            }
            self.ts.client_window.apply(&mut update);
            if let Some((address, data)) = update.memory {
                let viewer = &mut self.ts.memory_viewer_window;
                if viewer.address == address && viewer.size as usize == data.len() {
                    viewer.set_data(&data);
                }
            }
            if let Some(notice) = update.notice {
                self.ts.error_msg = None;
                self.gs.toasts.success(notice).duration(NORMAL_NOTIFICATION_TIME);
            }
        }
    }

    /// Translate UI events into owned worker requests. Never wait here.
    fn handle_events(&mut self, _c_ctx: &egui::Context) {
        while let Some(event) = self.ts.event_queue.pop() {
            if self.ts.client_ctx.is_none() {
                continue;
            }
            match event {
                GuiEvent::ResetState => {
                    self.ts.client_window.reset_state();
                    self.ts.disassembly_window.clear();
                    self.ts.final_register_window.clear_readout();
                }
                GuiEvent::Disconnect => {
                    if let Some(worker) = &mut self.ts.serial_worker {
                        if let Err(error) = worker.disconnect() {
                            self.gs.toasts.error(error.to_string());
                        }
                    }
                }
                GuiEvent::LoadRegisters | GuiEvent::RunProgram => {
                    let run = matches!(event, GuiEvent::RunProgram);
                    let regs = self.ts.initial_register_window.load_regs();
                    let mut uploads = Vec::new();
                    if run {
                        for blob in self.ts.resource_manager.blobs() {
                            let address = match blob.mount_address {
                                MountAddress::FlatAddress(address) => address,
                                MountAddress::CsIp => self.ts.initial_register_window.code_address(),
                            };
                            uploads.push(MemoryUpload {
                                name: blob.name.clone(),
                                address,
                                data: blob.data.clone(),
                                is_program: matches!(blob.blob_type, BinaryBlobType::Program),
                            });
                        }
                    }
                    if self.submit_serial(SerialCommand::Load { regs, run, uploads }) {
                        self.ts.client_window.reset_state();
                        self.ts.disassembly_window.clear();
                        self.ts.final_register_window.clear_readout();
                    }
                }
                GuiEvent::Step => {
                    if self.submit_serial(SerialCommand::Step) {
                        self.ts.disassembly_window.execution_started();
                    }
                }
                GuiEvent::StepInstruction => {
                    if self.submit_serial(SerialCommand::StepInstruction) {
                        self.ts.disassembly_window.execution_started();
                    }
                }
                GuiEvent::ResetCpu => {
                    if self.submit_serial(SerialCommand::ResetCpu) {
                        self.ts.disassembly_window.clear();
                    }
                }
                GuiEvent::ContinueTrap => {
                    if self.submit_serial(SerialCommand::ContinueTrap) {
                        self.ts.disassembly_window.execution_started();
                    }
                }
                GuiEvent::SetTrapEndpoint { cs, ip, enabled } => {
                    self.submit_serial(SerialCommand::SetTrapEndpoint { cs, ip, enabled });
                }
                GuiEvent::SetFlag { flag, enabled } => {
                    if self.submit_serial(SerialCommand::SetFlag { flag, enabled })
                        && flag == ServerFlags::EXECUTE_AUTOMATIC && enabled {
                        self.ts.disassembly_window.execution_started();
                    }
                }
                GuiEvent::SetExecutionMode(mode) => {
                    self.submit_serial(SerialCommand::SetExecutionMode(mode));
                }
                GuiEvent::WritePin { pin, value } => {
                    self.submit_serial(SerialCommand::WritePin { pin, value });
                }
                GuiEvent::WriteDataBus(value) => {
                    self.submit_serial(SerialCommand::WriteDataBus(value));
                }
                GuiEvent::EraseMemory => {
                    self.submit_serial(SerialCommand::EraseMemory);
                }
                GuiEvent::ClearCycleLog => {
                    self.submit_serial(SerialCommand::ClearCycleLog);
                }
                GuiEvent::ReadMemory { address, size } => {
                    self.submit_serial(SerialCommand::ReadMemory { address, size });
                }
                GuiEvent::PollStatus => {
                    self.ts.poll_requested = true;
                }
                GuiEvent::RefreshMemory => {
                    self.ts.memory_refresh_requested = self.ts.auto_refresh;
                }
                GuiEvent::UploadBlob {
                    blob_name,
                    mount_address,
                    size,
                } => {
                    if let Some(blob) = self.ts.resource_manager.blob(&blob_name) {
                        let address = match mount_address {
                            MountAddress::FlatAddress(address) => address,
                            MountAddress::CsIp => self.ts.initial_register_window.code_address(),
                        };
                        let size = size.unwrap_or(blob.data.len()).min(blob.data.len());
                        self.submit_serial(SerialCommand::Upload {
                            upload: MemoryUpload {
                                name: blob.name.clone(),
                                address,
                                data: blob.data[..size].to_vec(),
                                is_program: matches!(blob.blob_type, BinaryBlobType::Program),
                            },
                            regs: self.ts.initial_register_window.load_regs(),
                        });
                    }
                    else {
                        self.gs.toasts.error(format!("Blob {blob_name} not found."));
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
                                        if let Ok(()) = self.ts.resource_manager.update_blob(&program_name, &binary) {
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
                                    log::error!("Assembly failed for code editor {}: {}: {}", program_name, e, stderr);
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
                GuiEvent::ToggleRefreshMemory {
                    enabled: state,
                    hertz: rate,
                } => {
                    if state != self.ts.auto_refresh {
                        if state {
                            let new_event = ScheduledEvent::new(
                                ScheduleType::Repeat,
                                GuiEvent::RefreshMemory,
                                1000 / rate.max(1) as u64,
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
                        if !state {
                            self.ts.memory_refresh_requested = false;
                        }
                    }
                }
            }
        }
    }

    /// Coalesce background requests and alternate them so memory refresh cannot
    /// starve status updates (or vice versa). User actions run before this.
    fn submit_background(&mut self) {
        if self.ts.client_ctx.is_none() || self.serial_busy() {
            return;
        }
        if let Some(request) = self.ts.disassembly_window.pending_request() {
            if self.ts.serial_worker.as_mut().unwrap()
                .submit_background(SerialCommand::ReadDisassembly(request)).is_ok() {
                self.ts.disassembly_window.submitted(request);
            }
            return;
        }
        if self.ts.memory_refresh_requested && (!self.ts.poll_requested || self.ts.next_background_memory) {
            let viewer = &self.ts.memory_viewer_window;
            if self
                .ts
                .serial_worker
                .as_mut()
                .unwrap()
                .submit_background(SerialCommand::ReadMemory {
                    address: viewer.address,
                    size:    viewer.size,
                })
                .is_ok()
            {
                self.ts.memory_refresh_requested = false;
                self.ts.next_background_memory = false;
            }
        }
        else if self.ts.poll_requested && self.submit_serial(SerialCommand::Poll) {
            self.ts.poll_requested = false;
            self.ts.next_background_memory = true;
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

#[cfg(test)]
mod serial_scheduling_tests {
    use super::*;
    use crate::serial_worker::SerialReply;
    use arduinox86_client::{ProgramState, ServerCpuType};
    use std::{thread, time::Instant};

    #[test]
    fn tracks_and_resets_register_history() {
        use arduinox86_client::{trap::{InstructionState, TrapStatus}, RemoteCpuRegistersV2};
        let mut app = App::default();
        app.ts.client_ctx = Some(ClientContext::new(
            "test".into(), ServerCpuType::Intel80286, false,
            ProgramState::Execute, ServerFlags::ENABLE_TRAP_EXECUTION,
        ));
        let initial = RemoteCpuRegisters::V2(RemoteCpuRegistersV2::default());
        app.apply_serial_update(SerialUpdate { loaded_regs: Some(initial.clone()), ..Default::default() });
        assert_eq!(app.ts.final_register_window.readout_state_type(), CpuStateType::Current);
        let mut wire = [0; InstructionState::WIRE_SIZE];
        wire[0] = 1;
        let mut state = InstructionState::from_bytes(&wire).unwrap();
        state.status = TrapStatus::Paused;
        state.run_id = 1;
        for sequence in 1..=2 {
            state.sequence = sequence;
            state.capture_cycle = sequence * 10;
            let regs = RemoteCpuRegisters::V2(RemoteCpuRegistersV2 { ax: sequence as u16, ..Default::default() });
            app.apply_serial_update(SerialUpdate {
                instruction: Some(state.clone()), registers: Some((initial.clone(), regs)), ..Default::default()
            });
            let RemoteCpuRegisters::V2(baseline) = app.ts.final_register_window.regs() else { panic!("286 expected"); };
            assert_eq!(baseline.ax, (sequence - 1) as u16);
            assert_eq!(app.ts.final_register_window.readout_state_type(), CpuStateType::Current);
        }
        // An ordinary status poll must neither advance nor erase the baseline.
        app.apply_serial_update(SerialUpdate { instruction: Some(state.clone()), ..Default::default() });
        let RemoteCpuRegisters::V2(baseline) = app.ts.final_register_window.regs() else { panic!("286 expected"); };
        assert_eq!(baseline.ax, 1);
        state.status = TrapStatus::Captured;
        app.apply_serial_update(SerialUpdate {
            instruction: Some(state.clone()),
            registers: Some((initial.clone(), RemoteCpuRegisters::V2(RemoteCpuRegistersV2 { ax: 2, ..Default::default() }))),
            ..Default::default()
        });
        assert_eq!(app.ts.final_register_window.readout_state_type(), CpuStateType::Final);
        let RemoteCpuRegisters::V2(baseline) = app.ts.final_register_window.regs() else { panic!("286 expected"); };
        assert_eq!(baseline.ax, 0);

        // Even a repeated run/sequence identity cannot carry history across Load.
        let loaded = RemoteCpuRegisters::V2(RemoteCpuRegistersV2 { ax: 9, ..Default::default() });
        app.apply_serial_update(SerialUpdate { loaded_regs: Some(loaded.clone()), ..Default::default() });
        state.status = TrapStatus::Paused;
        app.apply_serial_update(SerialUpdate {
            instruction: Some(state.clone()), registers: Some((loaded.clone(), loaded.clone())), ..Default::default()
        });
        let RemoteCpuRegisters::V2(baseline) = app.ts.final_register_window.regs() else { panic!("286 expected"); };
        assert_eq!(baseline.ax, 9);
        state.status = TrapStatus::Idle;
        app.apply_serial_update(SerialUpdate { instruction: Some(state.clone()), ..Default::default() });
        assert!(!app.ts.final_register_window.open());
        state.status = TrapStatus::Paused;
        state.run_id += 1;
        app.apply_serial_update(SerialUpdate {
            instruction: Some(state), registers: Some((initial.clone(), loaded)), ..Default::default()
        });
        let RemoteCpuRegisters::V2(baseline) = app.ts.final_register_window.regs() else { panic!("286 expected"); };
        assert_eq!(baseline.ax, 0);
    }

    #[test]
    fn coalesces_and_schedules_requests() {
        let ctx = egui::Context::default();
        let (observed_tx, observed_rx) = crossbeam_channel::unbounded();
        let worker = SerialWorker::spawn(ctx.clone(), move |requests, replies, _, _| {
            while let Ok(command) = requests.recv() {
                observed_tx.send(command.label()).unwrap();
                if replies
                    .send(SerialReply {
                        operation: command.label(),
                        result:    Ok(SerialOutcome::Update(Default::default())),
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        let mut app = App::default();
        app.ts.serial_worker = Some(worker);
        app.ts.client_ctx = Some(ClientContext::new(
            "test".into(),
            ServerCpuType::Intel80286,
            false,
            ProgramState::Execute,
            0,
        ));
        app.ts.auto_refresh = true;
        for _ in 0..100 {
            app.ts.event_queue.push(GuiEvent::PollStatus);
            app.ts.event_queue.push(GuiEvent::RefreshMemory);
        }
        app.handle_events(&ctx);
        app.submit_background();
        assert_eq!(
            observed_rx.recv_timeout(Duration::from_secs(1)).unwrap(),
            "Reading CPU status"
        );

        // More timer ticks while the response is pending create no queued I/O.
        for _ in 0..100 {
            app.ts.event_queue.push(GuiEvent::PollStatus);
            app.ts.event_queue.push(GuiEvent::RefreshMemory);
        }
        app.handle_events(&ctx);
        app.submit_background();
        assert!(observed_rx.try_recv().is_err());
        assert!(app.ts.poll_requested && app.ts.memory_refresh_requested);

        let deadline = Instant::now() + Duration::from_secs(1);
        while app.serial_busy() {
            app.handle_serial_replies();
            assert!(Instant::now() < deadline);
            thread::yield_now();
        }
        app.submit_background();
        assert_eq!(
            observed_rx.recv_timeout(Duration::from_secs(1)).unwrap(),
            "Reading memory"
        );
        assert!(app.ts.poll_requested);
        assert!(!app.ts.memory_refresh_requested);
    }
}
