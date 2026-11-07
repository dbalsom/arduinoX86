use crate::client::ClientContext;

/// A fixed-height, single-line status bar. Truncation keeps long messages from
/// resizing the window or moving the other sections.
pub fn show(ctx: &egui::Context, connection: Option<&ClientContext>, activity: Option<&str>) {
    egui::TopBottomPanel::bottom("serial_status_bar")
        .resizable(false)
        .exact_height(26.0)
        .frame(egui::Frame::side_top_panel(&ctx.style()).inner_margin(egui::Margin::symmetric(6, 3)))
        .show(ctx, |ui| {
            let port = connection.map_or("Disconnected".into(), |client| format!("Port: {}", client.port_name));
            let baud = match connection.and_then(|client| client.baud_rate) {
                Some(0) => "Baud: USB".into(),
                Some(rate) => format!("Baud: {rate}"),
                None => "Baud: —".into(),
            };
            let cpu = connection.map_or("CPU: —".into(), |client| format!("CPU: {}", client.cpu_type));
            let state = connection.map_or("State: —".into(), |client| format!("State: {:?}", client.program_state));
            // Place connection details from the right edge, using only each
            // label's natural width. Activity gets the remaining space on the left.
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                for (index, text) in [baud, port, state, cpu].into_iter().enumerate() {
                    let tooltip = if index == 0 && connection.and_then(|client| client.baud_rate) == Some(0) {
                        "Native USB serial; a UART baud rate does not apply."
                    }
                    else {
                        &text
                    };
                    ui.add(egui::Label::new(&text).truncate()).on_hover_text(tooltip);
                    ui.separator();
                }
                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                    if let Some(activity) = activity {
                        ui.add(egui::Spinner::new().size(14.0));
                        ui.add(egui::Label::new(format!("{activity}…")).truncate())
                            .on_hover_text(activity);
                    }
                });
            });
        });
}
