use std::sync::{Arc, Mutex};

use eframe::egui::{
    self, Align2, Color32, FontId, Pos2, Rect, RichText, Rounding, Sense, Stroke, Vec2,
};

use crate::sim::{
    BenchMode, CLEARANCE_VOL_M3, CONROD_M, CRANK_RADIUS_M, DISP_VOL_M3, DYNO_BINS, DYNO_MAX_RPM,
    DYNO_MIN_RPM, PV_POINTS, SCOPE_LEN, SharedControls, STROKE_M, StrokePhase, TelemetrySnapshot,
};

const BG_PANEL: Color32 = Color32::from_rgb(24, 27, 32);
const BG_CANVAS: Color32 = Color32::from_rgb(17, 19, 22);
const BORDER_SUBTLE: Color32 = Color32::from_rgb(40, 45, 53);
const TEXT_PRIMARY: Color32 = Color32::from_rgb(225, 230, 238);
const TEXT_MUTED: Color32 = Color32::from_rgb(132, 140, 152);
const ACCENT_GREEN: Color32 = Color32::from_rgb(72, 188, 120);
const ACCENT_AMBER: Color32 = Color32::from_rgb(218, 154, 62);
const ACCENT_RED: Color32 = Color32::from_rgb(208, 72, 72);
const ACCENT_CYAN: Color32 = Color32::from_rgb(92, 178, 228);

#[derive(Clone, Copy, PartialEq, Eq)]
enum CutawayView {
    VPair,
    BankLongitudinal,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DiagnosticPlot {
    PvDiagram,
    Waveform,
}

pub struct FormulaApp {
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
    cutaway_view: CutawayView,
    diag_plot: DiagnosticPlot,
    space_was_down: bool,
    visuals_applied: bool,
}

impl FormulaApp {
    pub fn new(
        controls: Arc<Mutex<SharedControls>>,
        telemetry: Arc<Mutex<TelemetrySnapshot>>,
    ) -> Self {
        Self {
            controls,
            telemetry,
            cutaway_view: CutawayView::VPair,
            diag_plot: DiagnosticPlot::PvDiagram,
            space_was_down: false,
            visuals_applied: false,
        }
    }

    fn apply_industrial_style(&mut self, ctx: &egui::Context) {
        if self.visuals_applied {
            return;
        }
        self.visuals_applied = true;
        let mut style = (*ctx.style()).clone();
        style.visuals.window_fill = BG_CANVAS;
        style.visuals.panel_fill = BG_PANEL;
        style.visuals.extreme_bg_color = BG_CANVAS;
        style.visuals.faint_bg_color = Color32::from_rgb(30, 34, 40);
        style.visuals.widgets.noninteractive.bg_fill = BG_PANEL;
        style.visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.inactive.bg_fill = Color32::from_rgb(34, 38, 45);
        style.visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.hovered.bg_fill = Color32::from_rgb(44, 50, 59);
        style.visuals.widgets.active.bg_fill = Color32::from_rgb(56, 63, 74);
        style.spacing.item_spacing = Vec2::new(8.0, 6.0);
        ctx.set_style(style);
    }

    fn handle_keyboard(&mut self, ctx: &egui::Context, ctrl: &mut SharedControls) {
        let dt = ctx.input(|i| i.unstable_dt).clamp(0.001, 0.05) as f64;
        ctx.input(|i| {
            let space = i.key_down(egui::Key::Space);
            if space {
                ctrl.throttle = 1.0;
                self.space_was_down = true;
            } else if self.space_was_down {
                ctrl.throttle = 0.0;
                self.space_was_down = false;
            }

            if i.key_down(egui::Key::ArrowUp) || i.key_down(egui::Key::W) {
                ctrl.throttle = (ctrl.throttle + 1.8 * dt).clamp(0.0, 1.0);
            }
            if i.key_down(egui::Key::ArrowDown) || i.key_down(egui::Key::S) {
                ctrl.throttle = (ctrl.throttle - 2.2 * dt).clamp(0.0, 1.0);
            }
            if i.key_pressed(egui::Key::ArrowRight) || i.key_pressed(egui::Key::X) {
                ctrl.requested_gear = (ctrl.requested_gear + 1).min(7);
            }
            if i.key_pressed(egui::Key::ArrowLeft) || i.key_pressed(egui::Key::Z) {
                ctrl.requested_gear = ctrl.requested_gear.saturating_sub(1).max(1);
            }
            if i.key_pressed(egui::Key::Q) {
                ctrl.quick_start_trigger = true;
            }
        });
    }
}

impl eframe::App for FormulaApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_industrial_style(ctx);
        ctx.request_repaint();

        let snap = { self.telemetry.lock().unwrap().clone() };
        let mut ctrl = { *self.controls.lock().unwrap() };

        self.handle_keyboard(ctx, &mut ctrl);

        // 1. TOP TELEMETRY BAR (56px)
        egui::TopBottomPanel::top("top_bar")
            .exact_height(56.0)
            .show(ctx, |ui| {
                draw_top_bar(ui, &snap, &mut ctrl);
            });

        // 2. LEFT PANEL (255px) — 4-STEP STARTUP + THROTTLE & MODE + COLLAPSIBLE ECU
        egui::SidePanel::left("left_controls")
            .resizable(false)
            .exact_width(255.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    draw_left_panel(ui, &snap, &mut ctrl);
                });
            });

        // 3. RIGHT PANEL (340px) — DYNO CURVE + SWITCHABLE P-V/WAVEFORM + VITALS
        egui::SidePanel::right("right_telemetry")
            .resizable(false)
            .exact_width(340.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    draw_right_panel(ui, &snap, &mut ctrl, &mut self.diag_plot);
                });
            });

        // 4. CENTER PANEL — FULL-HEIGHT ENGINE CUTAWAY
        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(BG_CANVAS).inner_margin(10.0))
            .show(ctx, |ui| {
                draw_center_cutaway(ui, &snap, &mut self.cutaway_view);
            });

        *self.controls.lock().unwrap() = ctrl;
    }
}

fn draw_top_bar(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    ui.horizontal_centered(|ui| {
        // Left: Title + Precision Shift Strip
        ui.vertical(|ui| {
            ui.add_space(4.0);
            ui.label(
                RichText::new("F1 V10 DYNO")
                    .strong()
                    .monospace()
                    .size(13.0)
                    .color(TEXT_PRIMARY),
            );
            let (led_rect, _) = ui.allocate_exact_size(Vec2::new(168.0, 10.0), Sense::hover());
            let painter = ui.painter();
            let rpm_start = 13_500.0;
            let rpm_end = ctrl.rev_limit_rpm;
            let flash_on = ((ui.input(|i| i.time) * 14.0) as i32) % 2 == 0;
            for i in 0..15 {
                let frac = i as f64 / 14.0;
                let thresh = rpm_start + frac * (rpm_end - rpm_start);
                let lit = snap.rpm >= thresh;
                let col = if !lit {
                    Color32::from_rgb(38, 42, 48)
                } else if i < 5 {
                    ACCENT_GREEN
                } else if i < 10 {
                    ACCENT_AMBER
                } else if snap.rpm > rpm_end - 450.0 && !flash_on {
                    Color32::from_rgb(45, 55, 75)
                } else {
                    ACCENT_CYAN
                };
                let x0 = led_rect.left() + i as f32 * 11.2;
                let seg = Rect::from_min_size(Pos2::new(x0, led_rect.top() + 1.0), Vec2::new(9.0, 7.0));
                painter.rect_filled(seg, Rounding::same(1.5), col);
            }
        });

        ui.separator();

        // Center: Clean Monospace Readouts
        top_readout(
            ui,
            "RPM",
            &format!("{:5.0}", snap.rpm),
            if snap.rev_limiter_active { ACCENT_RED } else { TEXT_PRIMARY },
        );
        ui.separator();
        top_readout(
            ui,
            "POWER",
            &format!("{:3.0} BHP", snap.power_bhp.max(0.0)),
            ACCENT_GREEN,
        );
        ui.separator();
        top_readout(
            ui,
            "TORQUE",
            &format!("{:3.0} Nm", snap.torque_nm.max(0.0)),
            ACCENT_AMBER,
        );
        ui.separator();
        let gear_str = if snap.gear == 0 {
            "N".to_string()
        } else {
            format!("{}", snap.gear)
        };
        top_readout(ui, "GEAR", &gear_str, ACCENT_CYAN);
        ui.separator();
        top_readout(ui, "SPEED", &format!("{:3.0} km/h", snap.speed_kmh), TEXT_PRIMARY);

        // Right: Status & Quick Actions
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.button("Reset").clicked() {
                ctrl.reset_repair_trigger = true;
            }
            if ui
                .button(RichText::new("Quick Start [Q]").color(ACCENT_GREEN))
                .clicked()
            {
                ctrl.quick_start_trigger = true;
            }

            ui.add_space(6.0);
            let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
            let (status_txt, status_col) = if snap.cold_seized {
                ("COLD SEIZED", ACCENT_RED)
            } else if snap.knock_active {
                ("DETONATION", ACCENT_RED)
            } else if snap.valve_float_active {
                ("VALVE FLOAT", ACCENT_AMBER)
            } else if snap.is_running {
                ("RUNNING", ACCENT_GREEN)
            } else if ctrl.starter_active {
                ("CRANKING", ACCENT_AMBER)
            } else if min_temp >= 60.0 && ctrl.n2_pressure_bar >= 100.0 && ctrl.armed() {
                ("READY TO CRANK", ACCENT_GREEN)
            } else if ctrl.preheat_on() && min_temp < 60.0 {
                ("WARMING", ACCENT_AMBER)
            } else {
                ("OFFLINE", TEXT_MUTED)
            };
            ui.label(
                RichText::new(status_txt)
                    .monospace()
                    .strong()
                    .size(11.5)
                    .color(status_col),
            );
        });
    });
}

fn top_readout(ui: &mut egui::Ui, label: &str, value: &str, color: Color32) {
    ui.vertical(|ui| {
        ui.add_space(4.0);
        ui.label(RichText::new(label).size(9.5).color(TEXT_MUTED));
        ui.label(RichText::new(value).monospace().strong().size(15.5).color(color));
    });
}

fn draw_left_panel(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    ui.add_space(6.0);

    // 1. GUIDED 4-STEP STARTUP SEQUENCE
    ui.label(RichText::new("STARTUP SEQUENCE").strong().size(11.0).color(TEXT_MUTED));
    ui.add_space(2.0);

    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
    let warm_ok = min_temp >= 60.0;
    let pneu_ok = ctrl.n2_pressure_bar >= 100.0;
    let armed_ok = ctrl.armed();

    // Step 1: Pre-Heat Block
    ui.horizontal(|ui| {
        ui.label(RichText::new("1  Pre-Heat Block").size(12.0));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("{:.0}°C / 60°C", min_temp))
                    .monospace()
                    .size(11.0)
                    .color(if warm_ok { ACCENT_GREEN } else { TEXT_MUTED }),
            );
        });
    });
    ui.horizontal(|ui| {
        let pre_on = ctrl.preheat_on();
        let (lbl, col) = if warm_ok {
            ("READY", ACCENT_GREEN)
        } else if pre_on {
            ("HEATING", ACCENT_AMBER)
        } else {
            ("OFF", TEXT_PRIMARY)
        };
        if ui
            .add_sized([84.0, 22.0], egui::Button::new(RichText::new(lbl).monospace().size(11.0).color(col)))
            .clicked()
        {
            ctrl.set_preheat(!pre_on);
        }
        let frac = ((min_temp - 22.0) / (60.0 - 22.0)).clamp(0.0, 1.0) as f32;
        ui.add(
            egui::ProgressBar::new(frac)
                .desired_width(142.0)
                .desired_height(6.0)
                .fill(if warm_ok { ACCENT_GREEN } else { ACCENT_AMBER }),
        );
    });

    ui.add_space(4.0);

    // Step 2: Valve Pneumatics
    ui.horizontal(|ui| {
        ui.label(RichText::new("2  Valve Pneumatics").size(12.0));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("{:.0} bar", ctrl.n2_pressure_bar))
                    .monospace()
                    .size(11.0)
                    .color(if pneu_ok { ACCENT_GREEN } else { TEXT_MUTED }),
            );
        });
    });
    if ui
        .add_sized(
            [236.0, 22.0],
            egui::Button::new(
                RichText::new(if pneu_ok { "165 BAR (READY)" } else { "PRESSURIZE N2" })
                    .monospace()
                    .size(11.0)
                    .color(if pneu_ok { ACCENT_GREEN } else { TEXT_PRIMARY }),
            ),
        )
        .clicked()
    {
        ctrl.toggle_pneumatics();
    }

    ui.add_space(4.0);

    // Step 3: Fuel & Ignition
    ui.label(RichText::new("3  Fuel & Ignition").size(12.0));
    if ui
        .add_sized(
            [236.0, 22.0],
            egui::Button::new(
                RichText::new(if armed_ok { "ARMED" } else { "DISARMED" })
                    .monospace()
                    .size(11.0)
                    .color(if armed_ok { ACCENT_GREEN } else { TEXT_PRIMARY }),
            ),
        )
        .clicked()
    {
        ctrl.toggle_arm();
    }

    ui.add_space(4.0);

    // Step 4: Hold Air Starter
    ui.label(RichText::new("4  Air Starter").size(12.0));
    let ready_to_crank = warm_ok && pneu_ok && armed_ok && !snap.is_running;
    let starter_fill = if ctrl.starter_active {
        Color32::from_rgb(130, 88, 32)
    } else if ready_to_crank {
        Color32::from_rgb(36, 84, 56)
    } else {
        Color32::from_rgb(34, 38, 45)
    };
    let starter_btn = ui.add_sized(
        [236.0, 28.0],
        egui::Button::new(
            RichText::new(if ctrl.starter_active {
                "CRANKING..."
            } else {
                "HOLD TO CRANK"
            })
            .monospace()
            .strong()
            .size(11.5)
            .color(if ready_to_crank || ctrl.starter_active {
                Color32::WHITE
            } else {
                TEXT_PRIMARY
            }),
        )
        .fill(starter_fill),
    );
    ctrl.starter_active = starter_btn.is_pointer_button_down_on();

    ui.separator();

    // 2. THROTTLE & LOAD
    ui.label(RichText::new("THROTTLE & LOAD").strong().size(11.0).color(TEXT_MUTED));
    ui.horizontal(|ui| {
        ui.label(
            RichText::new(format!("TPS {:3.0}%", ctrl.throttle * 100.0))
                .monospace()
                .size(12.0),
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button("100%").clicked() {
                ctrl.throttle = 1.0;
            }
            if ui.small_button("0%").clicked() {
                ctrl.throttle = 0.0;
            }
        });
    });
    ui.add(egui::Slider::new(&mut ctrl.throttle, 0.0..=1.0).show_value(false));
    ui.label(RichText::new("[Space] WOT blip  |  [W/S] Throttle").size(10.0).color(TEXT_MUTED));

    ui.add_space(4.0);
    ui.label(RichText::new("Mode").size(11.5).color(TEXT_MUTED));
    egui::Grid::new("mode_grid")
        .num_columns(2)
        .spacing([6.0, 4.0])
        .show(ui, |ui| {
            let modes = [
                (BenchMode::FreeRev, "Free Rev"),
                (BenchMode::DynoSweep, "Dyno Pull"),
                (BenchMode::TrackManualGearbox, "Gearbox"),
                (BenchMode::MonzaHotlapAuto, "Monza Lap"),
            ];
            for (idx, (m, label)) in modes.iter().enumerate() {
                let selected = ctrl.bench_mode == *m;
                if ui
                    .add_sized([114.0, 22.0], egui::SelectableLabel::new(selected, *label))
                    .clicked()
                {
                    ctrl.bench_mode = *m;
                    if *m == BenchMode::DynoSweep {
                        ctrl.start_dyno_sweep = true;
                    }
                }
                if idx % 2 == 1 {
                    ui.end_row();
                }
            }
        });

    ui.add_space(4.0);
    match ctrl.bench_mode {
        BenchMode::FreeRev => {
            ui.label(RichText::new("Unloaded neutral bench").size(11.0).color(TEXT_MUTED));
        }
        BenchMode::DynoSweep => {
            if ui.add_sized([236.0, 24.0], egui::Button::new("Run 5.5k–19.5k Sweep")).clicked() {
                ctrl.start_dyno_sweep = true;
            }
            if snap.dyno_sweep_running {
                ui.label(RichText::new("Sweeping at WOT...").size(11.0).color(ACCENT_GREEN));
            }
        }
        BenchMode::TrackManualGearbox => {
            ui.horizontal(|ui| {
                if ui.button("- Shift [Z]").clicked() {
                    ctrl.requested_gear = ctrl.requested_gear.saturating_sub(1).max(1);
                }
                ui.label(
                    RichText::new(format!("G{}", ctrl.requested_gear))
                        .monospace()
                        .strong(),
                );
                if ui.button("+ Shift [X]").clicked() {
                    ctrl.requested_gear = (ctrl.requested_gear + 1).min(7);
                }
            });
        }
        BenchMode::MonzaHotlapAuto => {
            ui.label(
                RichText::new(format!("{} ({:.0} m)", snap.monza_sector_name, snap.monza_dist_m))
                    .monospace()
                    .size(11.0)
                    .color(ACCENT_CYAN),
            );
        }
        BenchMode::DynoManualBrake => {
            ui.label(format!("Water Brake: {:.0}%", ctrl.dyno_brake_load * 100.0));
            ui.add(egui::Slider::new(&mut ctrl.dyno_brake_load, 0.0..=1.0).show_value(false));
        }
        BenchMode::DynoRpmHold => {
            ui.label(format!("Hold Target: {:.0} RPM", ctrl.dyno_hold_rpm));
            ui.add(
                egui::Slider::new(&mut ctrl.dyno_hold_rpm, 4000.0..=19500.0)
                    .step_by(250.0)
                    .show_value(false),
            );
        }
    }

    ui.separator();

    // 3. COLLAPSIBLE ADVANCED ECU & AUDIO DRAWER (CLOSED BY DEFAULT)
    egui::CollapsingHeader::new("Advanced ECU & Audio")
        .default_open(false)
        .show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui
                    .selectable_label(ctrl.bench_mode == BenchMode::DynoManualBrake, "Brake Load")
                    .clicked()
                {
                    ctrl.bench_mode = BenchMode::DynoManualBrake;
                }
                if ui
                    .selectable_label(ctrl.bench_mode == BenchMode::DynoRpmHold, "RPM Hold")
                    .clicked()
                {
                    ctrl.bench_mode = BenchMode::DynoRpmHold;
                }
            });
            if ctrl.bench_mode == BenchMode::DynoManualBrake {
                ui.add(egui::Slider::new(&mut ctrl.dyno_brake_load, 0.0..=1.0).text("Load"));
            } else if ctrl.bench_mode == BenchMode::DynoRpmHold {
                ui.add(
                    egui::Slider::new(&mut ctrl.dyno_hold_rpm, 4000.0..=19500.0)
                        .step_by(250.0)
                        .text("RPM"),
                );
            }

            let afr = ctrl.lambda_target * 14.7;
            ui.label(format!("Mixture: λ {:.2} ({:.1}:1)", ctrl.lambda_target, afr));
            ui.add(egui::Slider::new(&mut ctrl.lambda_target, 0.75..=1.20).show_value(false));

            ui.label(format!("Spark Offset: {:+.1}°", ctrl.spark_offset_deg));
            ui.add(egui::Slider::new(&mut ctrl.spark_offset_deg, -15.0..=15.0).show_value(false));

            ui.checkbox(&mut ctrl.trumpets_auto, "Auto Variable Trumpets");
            ui.add_enabled(
                !ctrl.trumpets_auto,
                egui::Slider::new(&mut ctrl.manual_trumpet_mm, 75.0..=135.0).text("mm"),
            );

            ui.label(format!("Rev Limiter: {:.0} RPM", ctrl.rev_limit_rpm));
            ui.add(
                egui::Slider::new(&mut ctrl.rev_limit_rpm, 14000.0..=21000.0)
                    .step_by(250.0)
                    .show_value(false),
            );

            ui.label(format!("Idle Target: {:.0} RPM", ctrl.idle_rpm_target));
            ui.add(
                egui::Slider::new(&mut ctrl.idle_rpm_target, 3200.0..=5500.0)
                    .step_by(100.0)
                    .show_value(false),
            );

            ui.label(format!(
                "N₂ Pressure: {:.0} bar (min {:.0})",
                ctrl.n2_pressure_bar, snap.required_n2_bar
            ));
            ui.add(egui::Slider::new(&mut ctrl.n2_pressure_bar, 0.0..=200.0).show_value(false));

            ui.horizontal(|ui| {
                let mute_lbl = if ctrl.muted { "Muted" } else { "Sound ON" };
                ui.toggle_value(&mut ctrl.muted, mute_lbl);
                ui.add(egui::Slider::new(&mut ctrl.master_volume, 0.0..=1.5).text("Vol"));
            });
        });
}

fn draw_right_panel(
    ui: &mut egui::Ui,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    diag_plot: &mut DiagnosticPlot,
) {
    ui.add_space(6.0);

    // 1. DYNO CURVE
    let (peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm) = compute_dyno_peaks(snap);
    ui.horizontal(|ui| {
        ui.label(RichText::new("DYNO CURVE").strong().size(11.0).color(TEXT_MUTED));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button("Clear").clicked() {
                ctrl.clear_dyno_trace = true;
            }
        });
    });
    ui.label(
        RichText::new(format!(
            "Peak: {:.0} BHP @ {:.0}  |  {:.0} Nm @ {:.0}",
            peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm
        ))
        .monospace()
        .size(10.5)
        .color(TEXT_PRIMARY),
    );
    draw_dyno_plot(ui, snap);

    ui.add_space(10.0);

    // 2. SWITCHABLE DIAGNOSTIC PLOT (P-V DIAGRAM | EXHAUST WAVEFORM)
    ui.horizontal(|ui| {
        if ui
            .selectable_label(*diag_plot == DiagnosticPlot::PvDiagram, "P-V DIAGRAM")
            .clicked()
        {
            *diag_plot = DiagnosticPlot::PvDiagram;
        }
        if ui
            .selectable_label(*diag_plot == DiagnosticPlot::Waveform, "EXHAUST WAVEFORM")
            .clicked()
        {
            *diag_plot = DiagnosticPlot::Waveform;
        }
    });
    match *diag_plot {
        DiagnosticPlot::PvDiagram => draw_pv_plot(ui, snap),
        DiagnosticPlot::Waveform => draw_scope_plot(ui, snap),
    }

    ui.add_space(10.0);

    // 3. COMPACT VITALS TABLE
    ui.label(RichText::new("VITALS").strong().size(11.0).color(TEXT_MUTED));
    ui.add_space(2.0);
    egui::Grid::new("vitals_table")
        .num_columns(2)
        .spacing([12.0, 5.0])
        .striped(true)
        .show(ui, |ui| {
            vital_row(
                ui,
                "Block / Coolant",
                &format!("{:.1} °C", snap.coolant_temp_c),
                snap.coolant_temp_c > 112.0,
            );
            vital_row(
                ui,
                "Oil Temp",
                &format!("{:.1} °C", snap.oil_temp_c),
                snap.oil_temp_c > 122.0,
            );
            vital_row(
                ui,
                "Exhaust EGT",
                &format!("{:.0} °C", snap.egt_c),
                snap.egt_c > 980.0,
            );
            vital_row(
                ui,
                "Plenum Pressure",
                &format!("{:.2} bar", snap.plenum_pressure_bar),
                false,
            );
            vital_row(
                ui,
                "Volumetric Eff.",
                &format!("{:.1} %", snap.ve_percent),
                false,
            );
            if snap.bore_health < 0.99 || snap.knock_count > 0 {
                vital_row(
                    ui,
                    "Bore Health / Knock",
                    &format!("{:.0}% / {}", snap.bore_health * 100.0, snap.knock_count),
                    snap.bore_health < 0.75 || snap.knock_active,
                );
            }
        });
}

fn vital_row(ui: &mut egui::Ui, label: &str, value: &str, warn: bool) {
    ui.label(RichText::new(label).size(11.5).color(TEXT_MUTED));
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        ui.label(
            RichText::new(value)
                .monospace()
                .size(11.5)
                .color(if warn { ACCENT_RED } else { TEXT_PRIMARY }),
        );
    });
    ui.end_row();
}

fn compute_dyno_peaks(snap: &TelemetrySnapshot) -> (f32, f32, f32, f32) {
    let mut pb = 0.0f32;
    let mut pb_r = 0.0f32;
    let mut pt = 0.0f32;
    let mut pt_r = 0.0f32;
    for d in &snap.dyno_curve {
        if d.recorded {
            if d.power_bhp > pb {
                pb = d.power_bhp;
                pb_r = d.rpm;
            }
            if d.torque_nm > pt {
                pt = d.torque_nm;
                pt_r = d.rpm;
            }
        }
    }
    (pb, pb_r, pt, pt_r)
}

fn draw_dyno_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(320.0, 170.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, BG_CANVAS);
    p.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let pad = rect.shrink(14.0);
    for i in 1..4 {
        let y = pad.bottom() - (i as f32 / 4.0) * pad.height();
        p.line_segment(
            [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
            Stroke::new(0.7_f32, Color32::from_rgb(28, 32, 38)),
        );
    }

    let rpm_span = (DYNO_MAX_RPM - DYNO_MIN_RPM) as f32;
    let mut bhp_pts = Vec::new();
    let mut nm_pts = Vec::new();
    for i in 0..DYNO_BINS {
        let pt = snap.dyno_curve[i];
        if pt.recorded {
            let nx = ((pt.rpm - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
            let x = pad.left() + nx * pad.width();
            let y_bhp = pad.bottom() - (pt.power_bhp / 1000.0).clamp(0.0, 1.0) * pad.height();
            let y_nm = pad.bottom() - (pt.torque_nm / 450.0).clamp(0.0, 1.0) * pad.height();
            bhp_pts.push(Pos2::new(x, y_bhp));
            nm_pts.push(Pos2::new(x, y_nm));
        }
    }
    if nm_pts.len() >= 2 {
        p.add(egui::Shape::line(nm_pts, Stroke::new(1.5_f32, ACCENT_AMBER)));
    }
    if bhp_pts.len() >= 2 {
        p.add(egui::Shape::line(bhp_pts, Stroke::new(1.9_f32, ACCENT_GREEN)));
    }

    if snap.rpm >= DYNO_MIN_RPM as f64 {
        let nx = (((snap.rpm as f32) - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
        let x = pad.left() + nx * pad.width();
        let y_bhp = pad.bottom() - ((snap.power_bhp as f32) / 1000.0).clamp(0.0, 1.0) * pad.height();
        p.circle_filled(Pos2::new(x, y_bhp), 3.4, TEXT_PRIMARY);
    }

    p.text(
        Pos2::new(pad.left() + 2.0, pad.top()),
        Align2::LEFT_TOP,
        "1000 BHP / 450 Nm",
        FontId::monospace(9.5),
        TEXT_MUTED,
    );
    p.text(
        Pos2::new(pad.right() - 2.0, pad.bottom() - 2.0),
        Align2::RIGHT_BOTTOM,
        "5k–20k RPM",
        FontId::monospace(9.5),
        TEXT_MUTED,
    );
}

fn draw_pv_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(320.0, 165.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, BG_CANVAS);
    p.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let pad = rect.shrink(14.0);
    let v_min = (CLEARANCE_VOL_M3 * 1e6) as f32 * 0.9;
    let v_max = ((CLEARANCE_VOL_M3 + DISP_VOL_M3) * 1e6) as f32 * 1.03;
    let p_max = 105.0f32;

    for i in 1..4 {
        let y = pad.bottom() - (i as f32 / 4.0) * pad.height();
        p.line_segment(
            [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
            Stroke::new(0.7_f32, Color32::from_rgb(28, 32, 38)),
        );
    }

    let map_pt = |v: f32, pr: f32| -> Pos2 {
        let nx = ((v - v_min) / (v_max - v_min)).clamp(0.0, 1.0);
        let ny = (pr / p_max).clamp(0.0, 1.0);
        Pos2::new(pad.left() + nx * pad.width(), pad.bottom() - ny * pad.height())
    };

    let mut pts = Vec::with_capacity(PV_POINTS + 1);
    for &(v, pr) in &snap.pv_trace {
        pts.push(map_pt(v, pr));
    }
    if let Some(&first) = pts.first() {
        pts.push(first);
    }
    p.add(egui::Shape::line(pts, Stroke::new(1.6_f32, ACCENT_CYAN)));

    let cur_v = ((CLEARANCE_VOL_M3 + snap.cylinders[0].piston_norm * DISP_VOL_M3) * 1e6) as f32;
    let cur_pt = map_pt(cur_v, snap.cylinders[0].pressure_bar as f32);
    p.circle_filled(cur_pt, 3.5, ACCENT_AMBER);

    p.text(
        Pos2::new(pad.left() + 2.0, pad.top()),
        Align2::LEFT_TOP,
        format!(
            "105 bar  ({:.1} bar | {:.0} K)",
            snap.cylinders[0].pressure_bar, snap.cylinders[0].temp_k
        ),
        FontId::monospace(9.5),
        TEXT_MUTED,
    );
    p.text(
        Pos2::new(pad.right() - 2.0, pad.bottom() - 2.0),
        Align2::RIGHT_BOTTOM,
        "Volume (cc)",
        FontId::monospace(9.5),
        TEXT_MUTED,
    );
}

fn draw_scope_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(320.0, 165.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, BG_CANVAS);
    p.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let pad = rect.shrink(10.0);
    let cy = pad.center().y;
    p.line_segment(
        [Pos2::new(pad.left(), cy), Pos2::new(pad.right(), cy)],
        Stroke::new(0.8_f32, Color32::from_rgb(32, 36, 44)),
    );

    let mut pts_l = Vec::with_capacity(SCOPE_LEN);
    let mut pts_r = Vec::with_capacity(SCOPE_LEN);
    let mut pts_a = Vec::with_capacity(SCOPE_LEN);
    for i in 0..SCOPE_LEN {
        let x = pad.left() + (i as f32 / (SCOPE_LEN - 1) as f32) * pad.width();
        let amp = pad.height() * 0.44;
        pts_l.push(Pos2::new(x, cy - snap.scope_left[i].clamp(-1.0, 1.0) * amp));
        pts_r.push(Pos2::new(x, cy - snap.scope_right[i].clamp(-1.0, 1.0) * amp));
        pts_a.push(Pos2::new(x, cy - snap.scope_audio[i].clamp(-1.0, 1.0) * amp));
    }
    p.add(egui::Shape::line(
        pts_l,
        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(92, 178, 228, 110)),
    ));
    p.add(egui::Shape::line(
        pts_r,
        Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(218, 154, 62, 110)),
    ));
    p.add(egui::Shape::line(pts_a, Stroke::new(1.4_f32, TEXT_PRIMARY)));
}

fn draw_center_cutaway(
    ui: &mut egui::Ui,
    snap: &TelemetrySnapshot,
    view: &mut CutawayView,
) {
    ui.horizontal(|ui| {
        if ui
            .selectable_label(*view == CutawayView::VPair, "V-Pair Cross Section")
            .clicked()
        {
            *view = CutawayView::VPair;
        }
        if ui
            .selectable_label(*view == CutawayView::BankLongitudinal, "5-Cylinder Bank")
            .clicked()
        {
            *view = CutawayView::BankLongitudinal;
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("Trumpets {:.1} mm", snap.trumpet_length_mm))
                    .monospace()
                    .size(11.0)
                    .color(TEXT_MUTED),
            );
        });
    });

    ui.add_space(4.0);
    let avail = ui.available_size();
    let (rect, _) = ui.allocate_exact_size(Vec2::new(avail.x, avail.y.max(280.0)), Sense::hover());

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 4.0, BG_CANVAS);
    painter.rect_stroke(rect, 4.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    match *view {
        CutawayView::VPair => paint_v_pair_cutaway(&painter, rect, snap),
        CutawayView::BankLongitudinal => paint_bank_cutaway(&painter, rect, snap),
    }
}

fn gas_color(cyl: &crate::sim::CylinderSnapshot) -> Color32 {
    if cyl.comb_intensity_or_spark() > 0.85 {
        return Color32::from_rgba_unmultiplied(255, 222, 115, 225);
    }
    match cyl.phase {
        StrokePhase::Intake => {
            let a = (70.0 + cyl.pressure_bar as f32 * 55.0).clamp(60.0, 170.0) as u8;
            Color32::from_rgba_unmultiplied(72, 170, 235, a)
        }
        StrokePhase::Compression => {
            let p_frac = (cyl.pressure_bar as f32 / 35.0).clamp(0.0, 1.0);
            let r = (80.0 + p_frac * 165.0) as u8;
            let g = (150.0 - p_frac * 20.0) as u8;
            let b = (215.0 - p_frac * 165.0) as u8;
            Color32::from_rgba_unmultiplied(r, g, b, 190)
        }
        StrokePhase::Power => {
            let i = cyl.combustion_intensity as f32;
            let r = 250;
            let g = (95.0 + i * 140.0) as u8;
            let b = (30.0 + i * 105.0) as u8;
            Color32::from_rgba_unmultiplied(r, g, b, (145.0 + i * 95.0) as u8)
        }
        StrokePhase::Exhaust => {
            let p_frac = (cyl.pressure_bar as f32 / 8.0).clamp(0.15, 1.0);
            Color32::from_rgba_unmultiplied(210, 65, 48, (105.0 + p_frac * 85.0) as u8)
        }
    }
}

trait CylVisExt {
    fn comb_intensity_or_spark(&self) -> f32;
}
impl CylVisExt for crate::sim::CylinderSnapshot {
    fn comb_intensity_or_spark(&self) -> f32 {
        if self.spark_firing {
            1.0
        } else {
            self.combustion_intensity as f32
        }
    }
}

fn header_glow_color(egt_c: f64) -> Color32 {
    let t = ((egt_c - 320.0) / 650.0).clamp(0.0, 1.0) as f32;
    let r = (68.0 + t * 180.0) as u8;
    let g = (72.0 + t * 82.0) as u8;
    let b = (80.0 - t * 52.0) as u8;
    Color32::from_rgb(r, g, b)
}

fn paint_v_pair_cutaway(p: &egui::Painter, rect: Rect, snap: &TelemetrySnapshot) {
    let scale = (rect.height().min(rect.width() * 0.88)) * 2.35;
    let bore_px = (crate::sim::BORE_M as f32) * scale;
    let r_px = (CRANK_RADIUS_M as f32) * scale;
    let l_px = (CONROD_M as f32) * scale;
    let stroke_px = (STROKE_M as f32) * scale;

    let crank_center = Pos2::new(rect.center().x, rect.bottom() - rect.height() * 0.24);

    p.circle_filled(crank_center, r_px * 2.55, Color32::from_rgb(28, 31, 38));
    p.circle_stroke(crank_center, r_px * 2.55, Stroke::new(1.5_f32, Color32::from_rgb(62, 68, 80)));

    let half_v = (crate::sim::V_ANGLE_DEG as f32 * 0.5).to_radians();
    let pin_angle = (snap.crank_angle_rad as f32) - half_v;
    let pin_pos = Pos2::new(
        crank_center.x + pin_angle.sin() * r_px,
        crank_center.y - pin_angle.cos() * r_px,
    );

    let cw_dir = Vec2::new(-pin_angle.sin(), pin_angle.cos());
    p.circle_filled(
        crank_center + cw_dir * (r_px * 0.72),
        r_px * 1.15,
        Color32::from_rgb(50, 56, 66),
    );

    let banks = [
        (0usize, -half_v, "CYL 1 (L)"),
        (5usize, half_v, "CYL 6 (R)"),
    ];
    let exhaust_col = header_glow_color(snap.egt_c);

    for &(cyl_idx, bank_angle, label) in &banks {
        let cyl = &snap.cylinders[cyl_idx];
        let u_axis = Vec2::new(bank_angle.sin(), -bank_angle.cos());
        let u_perp = Vec2::new(bank_angle.cos(), bank_angle.sin());

        let d = pin_pos - crank_center;
        let proj = d.dot(u_axis);
        let perp_d = d.dot(u_perp);
        let wrist_dist = proj + (l_px * l_px - perp_d * perp_d).max(1.0).sqrt();
        let wrist_pos = crank_center + u_axis * wrist_dist;

        let tdc_wrist = r_px + l_px;
        let piston_h = bore_px * 0.34;
        let crown_offset = piston_h * 0.45;
        let head_dist = tdc_wrist + crown_offset + (stroke_px * 0.12);
        let cyl_bot_dist = l_px - r_px - piston_h * 0.35;

        let quad_pts = [
            crank_center + u_axis * cyl_bot_dist - u_perp * (bore_px * 0.54),
            crank_center + u_axis * head_dist - u_perp * (bore_px * 0.54),
            crank_center + u_axis * head_dist + u_perp * (bore_px * 0.54),
            crank_center + u_axis * cyl_bot_dist + u_perp * (bore_px * 0.54),
        ];
        p.add(egui::Shape::convex_polygon(
            quad_pts.to_vec(),
            Color32::from_rgb(22, 25, 31),
            Stroke::new(2.0_f32, Color32::from_rgb(72, 79, 92)),
        ));

        let crown_dist = wrist_dist + crown_offset;
        if head_dist > crown_dist {
            let gas_quad = [
                crank_center + u_axis * crown_dist - u_perp * (bore_px * 0.49),
                crank_center + u_axis * head_dist - u_perp * (bore_px * 0.49),
                crank_center + u_axis * head_dist + u_perp * (bore_px * 0.49),
                crank_center + u_axis * crown_dist + u_perp * (bore_px * 0.49),
            ];
            p.add(egui::Shape::convex_polygon(
                gas_quad.to_vec(),
                gas_color(cyl),
                Stroke::NONE,
            ));
        }

        let in_sign = if bank_angle < 0.0 { 1.0 } else { -1.0 };
        let ex_sign = -in_sign;

        let head_center = crank_center + u_axis * head_dist;
        let in_port = head_center + u_perp * (in_sign * bore_px * 0.23);
        let ex_port = head_center + u_perp * (ex_sign * bore_px * 0.23);

        let ex_outer1 = ex_port + u_axis * (bore_px * 0.25) + u_perp * (ex_sign * bore_px * 0.35);
        let ex_outer2 = ex_outer1 + u_perp * (ex_sign * bore_px * 0.55) - u_axis * (bore_px * 0.35);
        p.line_segment([ex_port, ex_outer1], Stroke::new(bore_px * 0.22, exhaust_col));
        p.line_segment([ex_outer1, ex_outer2], Stroke::new(bore_px * 0.22, exhaust_col));

        let trumpet_px = (snap.trumpet_length_mm as f32 * 0.001) * scale * 1.15;
        let runner_top = in_port + u_axis * (bore_px * 0.32);
        let trumpet_top = runner_top + u_axis * trumpet_px;
        p.line_segment(
            [in_port, runner_top],
            Stroke::new(bore_px * 0.26, Color32::from_rgb(48, 53, 63)),
        );
        p.line_segment(
            [runner_top, trumpet_top],
            Stroke::new(bore_px * 0.24, Color32::from_rgb(33, 37, 45)),
        );
        let rim_l = trumpet_top - u_perp * (bore_px * 0.18);
        let rim_r = trumpet_top + u_perp * (bore_px * 0.18);
        p.line_segment([rim_l, rim_r], Stroke::new(4.5_f32, Color32::from_rgb(122, 134, 152)));

        let tps_ang = (snap.effective_throttle as f32) * 1.48;
        let plate_dir = u_perp * tps_ang.cos() + u_axis * tps_ang.sin();
        let throttle_c = in_port + u_axis * (bore_px * 0.18);
        p.line_segment(
            [throttle_c - plate_dir * (bore_px * 0.12), throttle_c + plate_dir * (bore_px * 0.12)],
            Stroke::new(3.0_f32, ACCENT_AMBER),
        );

        if cyl.fuel_injecting {
            p.line_segment(
                [in_port + u_axis * (bore_px * 0.12), in_port - u_axis * (bore_px * 0.08)],
                Stroke::new(3.5_f32, Color32::from_rgba_unmultiplied(92, 178, 228, 165)),
            );
        }

        let draw_valve = |port: Pos2, lift_norm: f64, head_w: f32, col: Color32| {
            let travel = (lift_norm as f32) * (bore_px * 0.14);
            let seat = port - u_axis * travel;
            let stem_top = port + u_axis * (bore_px * 0.36) - u_axis * travel;
            let barrel_bot = port + u_axis * (bore_px * 0.18);
            let barrel_top = port + u_axis * (bore_px * 0.42);
            p.line_segment([barrel_bot, barrel_top], Stroke::new(8.5_f32, Color32::from_rgb(60, 86, 118)));
            p.line_segment([seat, stem_top], Stroke::new(2.6_f32, col));
            p.line_segment([seat - u_perp * head_w, seat + u_perp * head_w], Stroke::new(4.0_f32, col));
        };
        draw_valve(
            in_port,
            cyl.intake_lift_norm,
            bore_px * 0.14,
            Color32::from_rgb(130, 200, 245),
        );
        draw_valve(
            ex_port,
            cyl.exhaust_lift_norm,
            bore_px * 0.115,
            Color32::from_rgb(240, 140, 95),
        );

        let spark_top = head_center + u_axis * (bore_px * 0.18);
        p.line_segment([head_center, spark_top], Stroke::new(3.8_f32, Color32::from_rgb(198, 204, 214)));
        if cyl.spark_firing {
            p.circle_filled(head_center - u_axis * 3.0, 7.0, Color32::from_rgb(120, 230, 255));
            p.circle_filled(head_center - u_axis * 3.0, 3.2, Color32::WHITE);
        }

        let p_top = crown_dist;
        let p_bot = wrist_dist - piston_h * 0.45;
        let piston_quad = [
            crank_center + u_axis * p_bot - u_perp * (bore_px * 0.47),
            crank_center + u_axis * p_top - u_perp * (bore_px * 0.47),
            crank_center + u_axis * p_top + u_perp * (bore_px * 0.47),
            crank_center + u_axis * p_bot + u_perp * (bore_px * 0.47),
        ];
        p.add(egui::Shape::convex_polygon(
            piston_quad.to_vec(),
            Color32::from_rgb(168, 176, 188),
            Stroke::new(1.2_f32, Color32::from_rgb(212, 218, 228)),
        ));

        p.line_segment([pin_pos, wrist_pos], Stroke::new(7.5_f32, Color32::from_rgb(130, 138, 152)));
        p.circle_filled(wrist_pos, 4.2, TEXT_PRIMARY);

        let lbl_pos = head_center + u_perp * (ex_sign * bore_px * 0.65) + u_axis * (bore_px * 0.25);
        p.text(
            lbl_pos,
            Align2::CENTER_CENTER,
            format!("{}\n{:.1} bar", label, cyl.pressure_bar),
            FontId::monospace(10.5),
            TEXT_PRIMARY,
        );
    }

    p.line_segment([crank_center, pin_pos], Stroke::new(11.0_f32, Color32::from_rgb(104, 114, 130)));
    p.circle_filled(pin_pos, 7.0, ACCENT_AMBER);
    p.circle_filled(crank_center, 8.5, Color32::from_rgb(190, 198, 210));
}

fn paint_bank_cutaway(p: &egui::Painter, rect: Rect, snap: &TelemetrySnapshot) {
    let pad = rect.shrink(28.0);
    let col_w = pad.width() / 5.0;
    let bore_px = (col_w * 0.56).min(pad.height() * 0.28);
    let crank_y = pad.bottom() - pad.height() * 0.20;
    let r_px = pad.height() * 0.11;
    let l_px = pad.height() * 0.34;

    p.line_segment(
        [Pos2::new(pad.left() + 18.0, crank_y), Pos2::new(pad.right() - 18.0, crank_y)],
        Stroke::new(5.5_f32, Color32::from_rgb(70, 76, 88)),
    );

    for i in 0..5 {
        let cyl = &snap.cylinders[i];
        let cx = pad.left() + (i as f32 + 0.5) * col_w;
        let theta = cyl.local_angle_rad as f32;

        let pin_pos = Pos2::new(cx + theta.sin() * r_px, crank_y - theta.cos() * r_px);
        let wrist_y = pin_pos.y - (l_px * l_px - (pin_pos.x - cx).powi(2)).max(1.0).sqrt();
        let wrist_pos = Pos2::new(cx, wrist_y);

        let head_y = crank_y - r_px - l_px - bore_px * 0.30;
        let cyl_bot_y = crank_y - l_px + r_px + bore_px * 0.15;

        let sleeve = Rect::from_min_max(
            Pos2::new(cx - bore_px * 0.52, head_y),
            Pos2::new(cx + bore_px * 0.52, cyl_bot_y),
        );
        p.rect_filled(sleeve, 2.0, Color32::from_rgb(22, 25, 31));
        p.rect_stroke(sleeve, 2.0, Stroke::new(1.8_f32, Color32::from_rgb(72, 79, 92)));

        let crown_y = wrist_y - bore_px * 0.18;
        if crown_y > head_y {
            let gas_rect = Rect::from_min_max(
                Pos2::new(cx - bore_px * 0.48, head_y + 2.0),
                Pos2::new(cx + bore_px * 0.48, crown_y),
            );
            p.rect_filled(gas_rect, 0.0, gas_color(cyl));
        }

        let in_y = head_y + (cyl.intake_lift_norm as f32) * (bore_px * 0.14);
        let ex_y = head_y + (cyl.exhaust_lift_norm as f32) * (bore_px * 0.14);
        p.line_segment(
            [Pos2::new(cx - bore_px * 0.34, in_y), Pos2::new(cx - bore_px * 0.10, in_y)],
            Stroke::new(3.2_f32, Color32::from_rgb(130, 200, 245)),
        );
        p.line_segment(
            [Pos2::new(cx + bore_px * 0.10, ex_y), Pos2::new(cx + bore_px * 0.34, ex_y)],
            Stroke::new(3.2_f32, Color32::from_rgb(240, 140, 95)),
        );

        if cyl.spark_firing {
            p.circle_filled(Pos2::new(cx, head_y + 4.0), 5.5, Color32::from_rgb(120, 230, 255));
        }

        let piston_rect = Rect::from_min_max(
            Pos2::new(cx - bore_px * 0.46, crown_y),
            Pos2::new(cx + bore_px * 0.46, wrist_y + bore_px * 0.16),
        );
        p.rect_filled(piston_rect, 2.0, Color32::from_rgb(168, 176, 188));

        p.line_segment([Pos2::new(cx, crank_y), pin_pos], Stroke::new(7.5_f32, Color32::from_rgb(100, 110, 124)));
        p.line_segment([pin_pos, wrist_pos], Stroke::new(6.0_f32, Color32::from_rgb(140, 148, 162)));
        p.circle_filled(pin_pos, 5.2, ACCENT_AMBER);
        p.circle_filled(wrist_pos, 3.6, Color32::WHITE);

        p.text(
            Pos2::new(cx, head_y - 16.0),
            Align2::CENTER_CENTER,
            format!("CYL {}\n{:.1} bar", i + 1, cyl.pressure_bar),
            FontId::monospace(10.5),
            TEXT_PRIMARY,
        );
    }
}
