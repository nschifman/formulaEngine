use std::sync::{Arc, Mutex};

use eframe::egui::{
    self, Align2, Color32, FontId, Pos2, Rect, RichText, Rounding, Sense, Stroke, Vec2,
};

use crate::sim::{
    BenchMode, CLEARANCE_VOL_M3, CONROD_M, CRANK_RADIUS_M, DISP_VOL_M3, DYNO_BINS, DYNO_MAX_RPM,
    DYNO_MIN_RPM, FIRING_ORDER, NUM_CYL, PV_POINTS, SCOPE_LEN, SharedControls, STROKE_M,
    StrokePhase, TelemetrySnapshot,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum CutawayView {
    VPair,
    BankLongitudinal,
}

pub struct FormulaApp {
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
    cutaway_view: CutawayView,
    space_was_down: bool,
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
            space_was_down: false,
        }
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
        // Keep rendering continuously at display refresh rate for smooth 19,000 RPM animation
        ctx.request_repaint();

        let snap = { self.telemetry.lock().unwrap().clone() };
        let mut ctrl = { *self.controls.lock().unwrap() };

        self.handle_keyboard(ctx, &mut ctrl);

        // 1. TOP TELEMETRY & SHIFT LIGHT BAR
        egui::TopBottomPanel::top("top_telemetry")
            .exact_height(74.0)
            .show(ctx, |ui| {
                draw_top_bar(ui, &snap, &mut ctrl);
            });

        // 2. BOTTOM DRIVETRAIN & DYNO BAR
        egui::TopBottomPanel::bottom("bottom_drivetrain")
            .exact_height(72.0)
            .show(ctx, |ui| {
                draw_bottom_bar(ui, &snap, &mut ctrl);
            });

        // 3. LEFT PANEL — TEST-CELL RIG & ECU TUNING
        egui::SidePanel::left("left_controls")
            .resizable(false)
            .exact_width(275.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    draw_left_panel(ui, &snap, &mut ctrl);
                });
            });

        // 4. RIGHT PANEL — REAL-TIME TELEMETRY WALL
        egui::SidePanel::right("right_telemetry")
            .resizable(false)
            .exact_width(390.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    draw_right_panel(ui, &snap, &mut ctrl);
                });
            });

        // 5. CENTER PANEL — SWITCHABLE 2D CUTAWAY + FIRING STRIP
        egui::CentralPanel::default().show(ctx, |ui| {
            draw_center_cutaway(ui, &snap, &mut self.cutaway_view);
        });

        *self.controls.lock().unwrap() = ctrl;
    }
}

fn draw_top_bar(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    ui.horizontal_centered(|ui| {
        ui.vertical(|ui| {
            ui.label(
                RichText::new("FORMULA 1 V10 TEST CELL")
                    .strong()
                    .size(15.0)
                    .color(Color32::from_rgb(230, 235, 245)),
            );
            ui.label(
                RichText::new("3.0L 72° V10 • Pneumatic Valves • 1D Waveguide")
                    .size(11.0)
                    .color(Color32::from_rgb(135, 145, 160)),
            );
        });

        ui.add_space(14.0);

        // Digital F1 LED Shift Lights (15 LEDs)
        let (led_rect, _) = ui.allocate_exact_size(Vec2::new(210.0, 24.0), Sense::hover());
        let painter = ui.painter();
        painter.rect_filled(led_rect, 5.0, Color32::from_rgb(16, 18, 22));
        let rpm_start = 13_500.0;
        let rpm_end = ctrl.rev_limit_rpm;
        let flash_on = ((ui.input(|i| i.time) * 14.0) as i32) % 2 == 0;
        for i in 0..15 {
            let frac = i as f64 / 14.0;
            let thresh = rpm_start + frac * (rpm_end - rpm_start);
            let lit = snap.rpm >= thresh;
            let color = if !lit {
                Color32::from_rgb(38, 42, 48)
            } else if i < 5 {
                Color32::from_rgb(40, 225, 95)
            } else if i < 10 {
                Color32::from_rgb(245, 55, 55)
            } else if snap.rpm > rpm_end - 450.0 && !flash_on {
                Color32::from_rgb(35, 45, 80)
            } else {
                Color32::from_rgb(55, 155, 255)
            };
            let cx = led_rect.left() + 12.0 + i as f32 * 13.3;
            let cy = led_rect.center().y;
            painter.circle_filled(Pos2::new(cx, cy), 4.6, color);
        }

        ui.add_space(12.0);

        metric_box(
            ui,
            "RPM",
            &format!("{:5.0}", snap.rpm),
            if snap.rev_limiter_active {
                Color32::from_rgb(255, 95, 95)
            } else {
                Color32::from_rgb(245, 245, 250)
            },
        );
        metric_box(
            ui,
            "POWER",
            &format!("{:4.0} BHP", snap.power_bhp.max(0.0)),
            Color32::from_rgb(75, 225, 145),
        );
        metric_box(
            ui,
            "TORQUE",
            &format!("{:4.0} Nm", snap.torque_nm.max(0.0)),
            Color32::from_rgb(250, 190, 75),
        );
        let gear_str = if snap.gear == 0 {
            "N".to_string()
        } else {
            format!("{}", snap.gear)
        };
        metric_box(
            ui,
            "GEAR",
            &gear_str,
            Color32::from_rgb(100, 195, 255),
        );
        metric_box(
            ui,
            "SPEED",
            &format!("{:3.0} km/h", snap.speed_kmh),
            Color32::from_rgb(215, 220, 230),
        );

        ui.add_space(8.0);

        // Status Badges
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                if snap.cold_seized {
                    badge(ui, "COLD SEIZED!", Color32::from_rgb(45, 150, 240));
                } else if snap.is_running {
                    badge(ui, "RUNNING", Color32::from_rgb(40, 175, 90));
                } else if ctrl.starter_active {
                    badge(ui, "CRANKING", Color32::from_rgb(225, 165, 45));
                } else {
                    badge(ui, "OFFLINE", Color32::from_rgb(80, 85, 95));
                }

                if snap.valve_float_active {
                    badge(ui, "VALVE FLOAT", Color32::from_rgb(225, 130, 35));
                }
                if snap.knock_active {
                    badge(ui, "KNOCK!", Color32::from_rgb(230, 50, 50));
                }
            });

            ui.horizontal(|ui| {
                if ui
                    .button(RichText::new("Quick Start (Q)").strong().color(Color32::from_rgb(130, 240, 165)))
                    .clicked()
                {
                    ctrl.quick_start_trigger = true;
                }
                if ui.button("Repair / Reset").clicked() {
                    ctrl.reset_repair_trigger = true;
                }
            });
        });
    });
}

fn metric_box(ui: &mut egui::Ui, label: &str, value: &str, color: Color32) {
    egui::Frame::none()
        .fill(Color32::from_rgb(22, 25, 30))
        .rounding(5.0)
        .inner_margin(egui::Margin::symmetric(9.0, 4.0))
        .show(ui, |ui| {
            ui.vertical(|ui| {
                ui.label(RichText::new(label).size(10.0).color(Color32::from_rgb(130, 140, 155)));
                ui.label(RichText::new(value).strong().monospace().size(16.0).color(color));
            });
        });
}

fn badge(ui: &mut egui::Ui, text: &str, fill: Color32) {
    egui::Frame::none()
        .fill(fill)
        .rounding(4.0)
        .inner_margin(egui::Margin::symmetric(6.0, 2.0))
        .show(ui, |ui| {
            ui.label(RichText::new(text).strong().size(10.5).color(Color32::WHITE));
        });
}

fn draw_left_panel(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    ui.add_space(4.0);
    ui.heading(RichText::new("Test-Cell Startup Rig").size(14.5));
    ui.label(
        RichText::new("F1 tolerances seize cold (<60°C). Pre-heat fluids & pressurize N₂ before cranking.")
            .size(11.0)
            .color(Color32::from_rgb(145, 155, 170)),
    );
    ui.add_space(4.0);

    ui.horizontal(|ui| {
        ui.toggle_value(&mut ctrl.coolant_heater_on, "Coolant Heater");
        ui.label(format!("{:.0}°C", snap.coolant_temp_c));
    });
    ui.horizontal(|ui| {
        ui.toggle_value(&mut ctrl.oil_heater_on, "Oil Pre-Heater");
        ui.label(format!("{:.0}°C", snap.oil_temp_c));
    });

    ui.add_space(4.0);
    ui.label(format!(
        "N₂ Valve Bottle: {:.0} bar (min {:.0})",
        ctrl.n2_pressure_bar, snap.required_n2_bar
    ));
    ui.add(egui::Slider::new(&mut ctrl.n2_pressure_bar, 0.0..=200.0).show_value(false));

    ui.horizontal(|ui| {
        ui.toggle_value(&mut ctrl.fuel_pump_on, "Fuel Pump 100 bar");
        ui.toggle_value(&mut ctrl.ignition_on, "Ignition");
    });

    ui.add_space(4.0);
    let starter_btn = ui.add_sized(
        [255.0, 30.0],
        egui::Button::new(
            RichText::new(if ctrl.starter_active {
                "CRANKING AIR STARTER..."
            } else {
                "HOLD: External Air Starter Gun"
            })
            .strong(),
        )
        .fill(if ctrl.starter_active {
            Color32::from_rgb(195, 115, 30)
        } else {
            Color32::from_rgb(45, 52, 65)
        }),
    );
    ctrl.starter_active = starter_btn.is_pointer_button_down_on();

    ui.separator();
    ui.heading(RichText::new("Live ECU & Induction").size(14.5));

    ui.horizontal(|ui| {
        ui.label(format!("Throttle (TPS): {:3.0}%", ctrl.throttle * 100.0));
        if ui.small_button("Idle").clicked() {
            ctrl.throttle = 0.0;
        }
        if ui.small_button("50%").clicked() {
            ctrl.throttle = 0.5;
        }
        if ui.small_button("WOT").clicked() {
            ctrl.throttle = 1.0;
        }
    });
    ui.add(egui::Slider::new(&mut ctrl.throttle, 0.0..=1.0).show_value(false));

    ui.label(format!("Idle Target: {:.0} RPM", ctrl.idle_rpm_target));
    ui.add(egui::Slider::new(&mut ctrl.idle_rpm_target, 3200.0..=5500.0).step_by(100.0).show_value(false));

    ui.label(format!("Rev Limiter: {:.0} RPM", ctrl.rev_limit_rpm));
    ui.add(egui::Slider::new(&mut ctrl.rev_limit_rpm, 14000.0..=21000.0).step_by(250.0).show_value(false));

    let afr = ctrl.lambda_target * 14.7;
    ui.label(format!("Fuel Mixture: λ {:.2} ({:.1}:1 AFR)", ctrl.lambda_target, afr));
    ui.add(egui::Slider::new(&mut ctrl.lambda_target, 0.75..=1.20).show_value(false));

    ui.label(format!("Spark Offset: {:+.1}°", ctrl.spark_offset_deg));
    ui.add(egui::Slider::new(&mut ctrl.spark_offset_deg, -15.0..=15.0).show_value(false));

    ui.add_space(4.0);
    ui.checkbox(&mut ctrl.trumpets_auto, "Auto Variable Intake Trumpets");
    ui.label(format!("Trumpet Length: {:.1} mm", snap.trumpet_length_mm));
    ui.add_enabled(
        !ctrl.trumpets_auto,
        egui::Slider::new(&mut ctrl.manual_trumpet_mm, 75.0..=135.0).show_value(false),
    );

    ui.separator();
    ui.heading(RichText::new("Audio Output").size(14.5));
    ui.horizontal(|ui| {
        let mute_lbl = if ctrl.muted { "Muted" } else { "Sound ON" };
        ui.toggle_value(&mut ctrl.muted, mute_lbl);
        ui.add(egui::Slider::new(&mut ctrl.master_volume, 0.0..=1.5).text("Vol"));
    });
}

fn draw_bottom_bar(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    ui.horizontal_centered(|ui| {
        ui.label(RichText::new("Bench Mode:").strong());
        let modes = [
            (BenchMode::FreeRev, "Free Rev"),
            (BenchMode::DynoManualBrake, "Dyno Brake"),
            (BenchMode::DynoRpmHold, "RPM Hold"),
            (BenchMode::DynoSweep, "Full Dyno Pull"),
            (BenchMode::TrackManualGearbox, "7-Spd Gearbox"),
            (BenchMode::MonzaHotlapAuto, "Monza Lap"),
        ];
        for (m, name) in modes {
            let active = ctrl.bench_mode == m;
            if ui
                .selectable_label(active, RichText::new(name).strong())
                .clicked()
            {
                ctrl.bench_mode = m;
                if m == BenchMode::DynoSweep {
                    ctrl.start_dyno_sweep = true;
                }
            }
        }

        ui.separator();

        match ctrl.bench_mode {
            BenchMode::FreeRev => {
                ui.label("Keys: [Space] WOT Blip  |  [W/S] Throttle  |  [Q] Quick Start");
            }
            BenchMode::DynoManualBrake => {
                ui.label(format!("Water Brake Load: {:.0}%", ctrl.dyno_brake_load * 100.0));
                ui.add(egui::Slider::new(&mut ctrl.dyno_brake_load, 0.0..=1.0).show_value(false));
            }
            BenchMode::DynoRpmHold => {
                ui.label(format!("Target Hold: {:.0} RPM", ctrl.dyno_hold_rpm));
                ui.add(
                    egui::Slider::new(&mut ctrl.dyno_hold_rpm, 4000.0..=19500.0)
                        .step_by(250.0)
                        .show_value(false),
                );
            }
            BenchMode::DynoSweep => {
                if ui.button("Run 5.5k–19.5k Sweep").clicked() {
                    ctrl.start_dyno_sweep = true;
                }
                if snap.dyno_sweep_running {
                    ui.label(RichText::new("SWEEPING AT WOT...").color(Color32::from_rgb(90, 230, 145)));
                }
            }
            BenchMode::TrackManualGearbox => {
                if ui.button("- Downshift (Z)").clicked() {
                    ctrl.requested_gear = ctrl.requested_gear.saturating_sub(1).max(1);
                }
                ui.label(
                    RichText::new(format!("Gear {}", ctrl.requested_gear))
                        .strong()
                        .size(15.0),
                );
                if ui.button("+ Upshift (X)").clicked() {
                    ctrl.requested_gear = (ctrl.requested_gear + 1).min(7);
                }
                ui.label(format!("|  Plenum: {:.2} bar", snap.plenum_pressure_bar));
            }
            BenchMode::MonzaHotlapAuto => {
                ui.label(
                    RichText::new(format!(
                        "{} ({:.0} / 5793 m)",
                        snap.monza_sector_name, snap.monza_dist_m
                    ))
                    .strong()
                    .color(Color32::from_rgb(100, 200, 255)),
                );
            }
        }
    });
}

fn draw_right_panel(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    // 1. P-V Indicator Diagram
    ui.horizontal(|ui| {
        ui.heading(RichText::new("P-V Diagram (Cyl 1)").size(14.0));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!(
                    "{:.1} bar | {:.0} K",
                    snap.cylinders[0].pressure_bar, snap.cylinders[0].temp_k
                ))
                    .monospace()
                    .color(Color32::from_rgb(90, 220, 255)),
            );
        });
    });
    draw_pv_plot(ui, snap);

    ui.add_space(6.0);

    // 2. Dyno Power & Torque Curve
    let (peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm) = compute_dyno_peaks(snap);
    ui.horizontal(|ui| {
        ui.heading(RichText::new("Dyno Curve").size(14.0));
        if ui.small_button("Clear").clicked() {
            ctrl.clear_dyno_trace = true;
        }
    });
    ui.label(
        RichText::new(format!(
            "Peak: {:.0} BHP @ {:.0} | {:.0} Nm @ {:.0}",
            peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm
        ))
        .size(11.0)
        .color(Color32::from_rgb(175, 190, 210)),
    );
    draw_dyno_plot(ui, snap);

    ui.add_space(6.0);

    // 3. Exhaust & Audio Oscilloscope
    ui.heading(RichText::new("Acoustic & Collector Scope").size(14.0));
    draw_scope_plot(ui, snap);

    ui.add_space(8.0);

    // 4. Engine Health & Fluid Telemetry Bars
    ui.heading(RichText::new("Thermal & Mechanical Health").size(14.0));
    health_bar(ui, "Coolant Temp", snap.coolant_temp_c, 20.0, 125.0, "°C", snap.coolant_temp_c > 112.0);
    health_bar(ui, "Oil Temp", snap.oil_temp_c, 20.0, 135.0, "°C", snap.oil_temp_c > 122.0);
    health_bar(ui, "Exhaust EGT", snap.egt_c, 25.0, 1080.0, "°C", snap.egt_c > 980.0);
    health_bar(ui, "Vol. Efficiency", snap.ve_percent, 0.0, 130.0, "%", false);
    health_bar(
        ui,
        "Bore / Ring Health",
        snap.bore_health * 100.0,
        0.0,
        100.0,
        "%",
        snap.bore_health < 0.65,
    );
    ui.label(
        RichText::new(format!(
            "Plenum: {:.2} bar  |  Knock Events: {}",
            snap.plenum_pressure_bar, snap.knock_count
        ))
        .size(11.5)
        .color(Color32::from_rgb(160, 170, 185)),
    );
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

fn health_bar(
    ui: &mut egui::Ui,
    label: &str,
    val: f64,
    min: f64,
    max: f64,
    unit: &str,
    warn: bool,
) {
    let frac = ((val - min) / (max - min)).clamp(0.0, 1.0) as f32;
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).size(11.5));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!("{:.1}{}", val, unit))
                    .monospace()
                    .size(11.5)
                    .color(if warn {
                        Color32::from_rgb(255, 85, 85)
                    } else {
                        Color32::from_rgb(215, 225, 235)
                    }),
            );
        });
    });
    let bar = egui::ProgressBar::new(frac)
        .desired_height(7.0)
        .fill(if warn {
            Color32::from_rgb(220, 55, 55)
        } else {
            Color32::from_rgb(65, 165, 235)
        });
    ui.add(bar);
    ui.add_space(2.0);
}

fn draw_pv_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(370.0, 155.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 6.0, Color32::from_rgb(14, 17, 22));
    p.rect_stroke(rect, 6.0, Stroke::new(1.0_f32, Color32::from_rgb(42, 48, 58)));

    let pad = rect.shrink(14.0);
    let v_min = (CLEARANCE_VOL_M3 * 1e6) as f32 * 0.9;
    let v_max = ((CLEARANCE_VOL_M3 + DISP_VOL_M3) * 1e6) as f32 * 1.03;
    let p_max = 105.0f32;

    // Subtle grid lines
    for i in 1..4 {
        let y = pad.bottom() - (i as f32 / 4.0) * pad.height();
        p.line_segment(
            [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
            Stroke::new(0.7_f32, Color32::from_rgb(30, 35, 44)),
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
    p.add(egui::Shape::line(
        pts,
        Stroke::new(1.7_f32, Color32::from_rgb(70, 210, 255)),
    ));

    // Current instantaneous dot for Cylinder 1
    let cur_v = ((CLEARANCE_VOL_M3 + snap.cylinders[0].piston_norm * DISP_VOL_M3) * 1e6) as f32;
    let cur_pt = map_pt(cur_v, snap.cylinders[0].pressure_bar as f32);
    p.circle_filled(cur_pt, 3.8, Color32::from_rgb(255, 195, 70));

    p.text(
        Pos2::new(pad.left() + 4.0, pad.top()),
        Align2::LEFT_TOP,
        "105 bar",
        FontId::monospace(9.5),
        Color32::from_rgb(115, 125, 140),
    );
    p.text(
        Pos2::new(pad.right() - 2.0, pad.bottom() - 2.0),
        Align2::RIGHT_BOTTOM,
        "Volume (cc)",
        FontId::monospace(9.5),
        Color32::from_rgb(115, 125, 140),
    );
}

fn draw_dyno_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(370.0, 160.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 6.0, Color32::from_rgb(14, 17, 22));
    p.rect_stroke(rect, 6.0, Stroke::new(1.0_f32, Color32::from_rgb(42, 48, 58)));

    let pad = rect.shrink(14.0);
    for i in 1..4 {
        let y = pad.bottom() - (i as f32 / 4.0) * pad.height();
        p.line_segment(
            [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
            Stroke::new(0.7_f32, Color32::from_rgb(30, 35, 44)),
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
        p.add(egui::Shape::line(
            nm_pts,
            Stroke::new(1.6_f32, Color32::from_rgb(250, 185, 65)),
        ));
    }
    if bhp_pts.len() >= 2 {
        p.add(egui::Shape::line(
            bhp_pts,
            Stroke::new(2.0_f32, Color32::from_rgb(70, 230, 145)),
        ));
    }

    // Current live operating point
    if snap.rpm >= DYNO_MIN_RPM as f64 {
        let nx = (((snap.rpm as f32) - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
        let x = pad.left() + nx * pad.width();
        let y_bhp = pad.bottom() - ((snap.power_bhp as f32) / 1000.0).clamp(0.0, 1.0) * pad.height();
        p.circle_filled(Pos2::new(x, y_bhp), 3.6, Color32::WHITE);
    }

    p.text(
        Pos2::new(pad.left() + 4.0, pad.top()),
        Align2::LEFT_TOP,
        "1000 BHP / 450 Nm",
        FontId::monospace(9.5),
        Color32::from_rgb(125, 135, 150),
    );
    p.text(
        Pos2::new(pad.right() - 2.0, pad.bottom() - 2.0),
        Align2::RIGHT_BOTTOM,
        "5k–20k RPM",
        FontId::monospace(9.5),
        Color32::from_rgb(125, 135, 150),
    );
}

fn draw_scope_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(370.0, 105.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 6.0, Color32::from_rgb(14, 17, 22));
    p.rect_stroke(rect, 6.0, Stroke::new(1.0_f32, Color32::from_rgb(42, 48, 58)));

    let pad = rect.shrink(8.0);
    let cy = pad.center().y;
    p.line_segment(
        [Pos2::new(pad.left(), cy), Pos2::new(pad.right(), cy)],
        Stroke::new(0.8_f32, Color32::from_rgb(35, 42, 52)),
    );

    let mut pts_l = Vec::with_capacity(SCOPE_LEN);
    let mut pts_r = Vec::with_capacity(SCOPE_LEN);
    let mut pts_a = Vec::with_capacity(SCOPE_LEN);
    for i in 0..SCOPE_LEN {
        let x = pad.left() + (i as f32 / (SCOPE_LEN - 1) as f32) * pad.width();
        let amp = pad.height() * 0.45;
        pts_l.push(Pos2::new(x, cy - snap.scope_left[i].clamp(-1.0, 1.0) * amp));
        pts_r.push(Pos2::new(x, cy - snap.scope_right[i].clamp(-1.0, 1.0) * amp));
        pts_a.push(Pos2::new(x, cy - snap.scope_audio[i].clamp(-1.0, 1.0) * amp));
    }
    p.add(egui::Shape::line(pts_l, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(70, 200, 255, 140))));
    p.add(egui::Shape::line(pts_r, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(255, 155, 65, 140))));
    p.add(egui::Shape::line(pts_a, Stroke::new(1.4_f32, Color32::from_rgb(235, 240, 245))));
}

fn draw_center_cutaway(
    ui: &mut egui::Ui,
    snap: &TelemetrySnapshot,
    view: &mut CutawayView,
) {
    ui.horizontal(|ui| {
        ui.heading(RichText::new("Live Engine Cutaway").size(15.0));
        ui.add_space(12.0);
        if ui
            .selectable_label(*view == CutawayView::VPair, "72° V-Pair (Cyl 1 & 6)")
            .clicked()
        {
            *view = CutawayView::VPair;
        }
        if ui
            .selectable_label(*view == CutawayView::BankLongitudinal, "5-Cyl Bank Slice (Cyl 1–5)")
            .clicked()
        {
            *view = CutawayView::BankLongitudinal;
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(
                RichText::new(format!(
                    "Trumpets: {:.1} mm  |  EGT: {:.0}°C",
                    snap.trumpet_length_mm, snap.egt_c
                ))
                .monospace()
                .size(12.0)
                .color(Color32::from_rgb(175, 190, 210)),
            );
        });
    });

    // Reserve bottom 82px for the 10-cylinder firing strip
    let avail = ui.available_size();
    let canvas_h = (avail.y - 86.0).max(260.0);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(avail.x, canvas_h), Sense::hover());

    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 8.0, Color32::from_rgb(15, 18, 24));
    painter.rect_stroke(rect, 8.0, Stroke::new(1.0_f32, Color32::from_rgb(42, 48, 60)));

    match *view {
        CutawayView::VPair => paint_v_pair_cutaway(&painter, rect, snap),
        CutawayView::BankLongitudinal => paint_bank_cutaway(&painter, rect, snap),
    }

    ui.add_space(6.0);
    draw_firing_strip(ui, snap);
}

fn gas_color(cyl: &crate::sim::CylinderSnapshot) -> Color32 {
    if cyl.comb_intensity_or_spark() > 0.85 {
        return Color32::from_rgba_unmultiplied(255, 225, 110, 230);
    }
    match cyl.phase {
        StrokePhase::Intake => {
            let a = (70.0 + cyl.pressure_bar as f32 * 55.0).clamp(60.0, 175.0) as u8;
            Color32::from_rgba_unmultiplied(65, 175, 255, a)
        }
        StrokePhase::Compression => {
            let p_frac = (cyl.pressure_bar as f32 / 35.0).clamp(0.0, 1.0);
            let r = (80.0 + p_frac * 170.0) as u8;
            let g = (150.0 - p_frac * 20.0) as u8;
            let b = (220.0 - p_frac * 170.0) as u8;
            Color32::from_rgba_unmultiplied(r, g, b, 195)
        }
        StrokePhase::Power => {
            let i = cyl.combustion_intensity as f32;
            let r = 255;
            let g = (95.0 + i * 145.0) as u8;
            let b = (30.0 + i * 110.0) as u8;
            Color32::from_rgba_unmultiplied(r, g, b, (150.0 + i * 95.0) as u8)
        }
        StrokePhase::Exhaust => {
            let p_frac = (cyl.pressure_bar as f32 / 8.0).clamp(0.15, 1.0);
            Color32::from_rgba_unmultiplied(220, 65, 45, (110.0 + p_frac * 90.0) as u8)
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
    let r = (75.0 + t * 180.0) as u8;
    let g = (75.0 + t * 85.0) as u8;
    let b = (82.0 - t * 55.0) as u8;
    Color32::from_rgb(r, g, b)
}

fn paint_v_pair_cutaway(p: &egui::Painter, rect: Rect, snap: &TelemetrySnapshot) {
    // Scale meters -> pixels cleanly inside canvas
    let scale = (rect.height().min(rect.width() * 0.88)) * 2.35; // ~ 1m = scale px; stroke 41mm, bore 96mm, rod 105mm
    let bore_px = (crate::sim::BORE_M as f32) * scale;
    let r_px = (CRANK_RADIUS_M as f32) * scale;
    let l_px = (CONROD_M as f32) * scale;
    let stroke_px = (STROKE_M as f32) * scale;

    let crank_center = Pos2::new(rect.center().x, rect.bottom() - rect.height() * 0.24);

    // Draw crankcase housing
    p.circle_filled(crank_center, r_px * 2.55, Color32::from_rgb(30, 34, 42));
    p.circle_stroke(crank_center, r_px * 2.55, Stroke::new(2.0_f32, Color32::from_rgb(68, 76, 90)));

    // Shared crankpin angle: Left bank at -V/2, Right bank at +V/2
    let half_v = (crate::sim::V_ANGLE_DEG as f32 * 0.5).to_radians();
    let pin_angle = (snap.crank_angle_rad as f32) - half_v;
    let pin_pos = Pos2::new(
        crank_center.x + pin_angle.sin() * r_px,
        crank_center.y - pin_angle.cos() * r_px,
    );

    // Draw crank counterweight opposite crankpin
    let cw_dir = Vec2::new(-pin_angle.sin(), pin_angle.cos());
    p.circle_filled(crank_center + cw_dir * (r_px * 0.72), r_px * 1.15, Color32::from_rgb(55, 62, 74));

    // Draw Left Bank (Cyl 1 = index 0, -V/2) and Right Bank (Cyl 6 = index 5, +V/2)
    let banks = [
        (0usize, -half_v, "CYL 1 (L)"),
        (5usize, half_v, "CYL 6 (R)"),
    ];

    let exhaust_col = header_glow_color(snap.egt_c);

    for &(cyl_idx, bank_angle, label) in &banks {
        let cyl = &snap.cylinders[cyl_idx];
        let u_axis = Vec2::new(bank_angle.sin(), -bank_angle.cos()); // from crank toward cylinder head
        let u_perp = Vec2::new(bank_angle.cos(), bank_angle.sin());  // perpendicular (rightward wrt cyl)

        // Exact geometric intersection of conrod from crankpin onto cylinder axis
        let d = pin_pos - crank_center;
        let proj = d.dot(u_axis);
        let perp_d = d.dot(u_perp);
        let wrist_dist = proj + (l_px * l_px - perp_d * perp_d).max(1.0).sqrt();
        let wrist_pos = crank_center + u_axis * wrist_dist;

        let tdc_wrist = r_px + l_px;
        let piston_h = bore_px * 0.34;
        let crown_offset = piston_h * 0.45;
        let head_dist = tdc_wrist + crown_offset + (stroke_px * 0.12); // includes clearance gap
        let cyl_bot_dist = l_px - r_px - piston_h * 0.35;

        // Cylinder bore sleeve & block walls
        let quad_pts = [
            crank_center + u_axis * cyl_bot_dist - u_perp * (bore_px * 0.54),
            crank_center + u_axis * head_dist - u_perp * (bore_px * 0.54),
            crank_center + u_axis * head_dist + u_perp * (bore_px * 0.54),
            crank_center + u_axis * cyl_bot_dist + u_perp * (bore_px * 0.54),
        ];
        p.add(egui::Shape::convex_polygon(
            quad_pts.to_vec(),
            Color32::from_rgb(24, 28, 35),
            Stroke::new(2.5_f32, Color32::from_rgb(78, 86, 102)),
        ));

        // Dynamic gas fill between piston crown and cylinder head
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

        // Intake runner & Variable sliding intake trumpet (inward V-side)
        let in_sign = if bank_angle < 0.0 { 1.0 } else { -1.0 }; // inner V valley
        let ex_sign = -in_sign;                                   // outer exhaust headers

        let head_center = crank_center + u_axis * head_dist;
        let in_port = head_center + u_perp * (in_sign * bore_px * 0.23);
        let ex_port = head_center + u_perp * (ex_sign * bore_px * 0.23);

        // Glowing Inconel Exhaust Header Tube
        let ex_outer1 = ex_port + u_axis * (bore_px * 0.25) + u_perp * (ex_sign * bore_px * 0.35);
        let ex_outer2 = ex_outer1 + u_perp * (ex_sign * bore_px * 0.55) - u_axis * (bore_px * 0.35);
        p.line_segment([ex_port, ex_outer1], Stroke::new(bore_px * 0.22, exhaust_col));
        p.line_segment([ex_outer1, ex_outer2], Stroke::new(bore_px * 0.22, exhaust_col));

        // Intake runner + sliding carbon trumpet
        let trumpet_px = (snap.trumpet_length_mm as f32 * 0.001) * scale * 1.15;
        let runner_top = in_port + u_axis * (bore_px * 0.32);
        let trumpet_top = runner_top + u_axis * trumpet_px;
        p.line_segment(
            [in_port, runner_top],
            Stroke::new(bore_px * 0.26, Color32::from_rgb(52, 58, 70)),
        );
        // Sliding trumpet sleeve
        p.line_segment(
            [runner_top, trumpet_top],
            Stroke::new(bore_px * 0.24, Color32::from_rgb(35, 40, 48)),
        );
        // Carbon bellmouth rim
        let rim_l = trumpet_top - u_perp * (bore_px * 0.18);
        let rim_r = trumpet_top + u_perp * (bore_px * 0.18);
        p.line_segment([rim_l, rim_r], Stroke::new(5.0_f32, Color32::from_rgb(130, 145, 165)));

        // Butterfly throttle plate inside runner
        let tps_ang = (snap.effective_throttle as f32) * 1.48;
        let plate_dir = u_perp * tps_ang.cos() + u_axis * tps_ang.sin();
        let throttle_c = in_port + u_axis * (bore_px * 0.18);
        p.line_segment(
            [throttle_c - plate_dir * (bore_px * 0.12), throttle_c + plate_dir * (bore_px * 0.12)],
            Stroke::new(3.0_f32, Color32::from_rgb(240, 185, 70)),
        );

        // Fuel injector spray mist
        if cyl.fuel_injecting {
            p.line_segment(
                [in_port + u_axis * (bore_px * 0.12), in_port - u_axis * (bore_px * 0.08)],
                Stroke::new(4.0_f32, Color32::from_rgba_unmultiplied(80, 210, 255, 170)),
            );
        }

        // Intake & Exhaust Valves + Pneumatic N2 Return Cylinders
        let draw_valve = |port: Pos2, lift_norm: f64, head_w: f32, col: Color32| {
            let travel = (lift_norm as f32) * (bore_px * 0.14);
            let seat = port - u_axis * travel;
            let stem_top = port + u_axis * (bore_px * 0.36) - u_axis * travel;
            // Pneumatic N2 spring barrel
            let barrel_bot = port + u_axis * (bore_px * 0.18);
            let barrel_top = port + u_axis * (bore_px * 0.42);
            p.line_segment([barrel_bot, barrel_top], Stroke::new(9.0_f32, Color32::from_rgb(65, 95, 130)));
            // Valve stem + head
            p.line_segment([seat, stem_top], Stroke::new(2.8_f32, col));
            p.line_segment(
                [seat - u_perp * head_w, seat + u_perp * head_w],
                Stroke::new(4.2_f32, col),
            );
        };
        draw_valve(
            in_port,
            cyl.intake_lift_norm,
            bore_px * 0.14,
            Color32::from_rgb(130, 210, 255),
        );
        draw_valve(
            ex_port,
            cyl.exhaust_lift_norm,
            bore_px * 0.115,
            Color32::from_rgb(255, 145, 95),
        );

        // Spark Plug + Electric Discharge Flash
        let spark_top = head_center + u_axis * (bore_px * 0.18);
        p.line_segment(
            [head_center, spark_top],
            Stroke::new(4.0_f32, Color32::from_rgb(210, 215, 225)),
        );
        if cyl.spark_firing {
            p.circle_filled(head_center - u_axis * 3.0, 7.5, Color32::from_rgb(120, 240, 255));
            p.circle_filled(head_center - u_axis * 3.0, 3.5, Color32::WHITE);
        }

        // Piston (short-skirt F1 slipper piston)
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
            Color32::from_rgb(175, 185, 198),
            Stroke::new(1.5_f32, Color32::from_rgb(225, 230, 240)),
        ));

        // Titanium Connecting Rod
        p.line_segment([pin_pos, wrist_pos], Stroke::new(8.0_f32, Color32::from_rgb(135, 145, 160)));
        p.circle_filled(wrist_pos, 4.5, Color32::from_rgb(230, 235, 240));

        // Cylinder label & live pressure
        let lbl_pos = head_center + u_perp * (ex_sign * bore_px * 0.65) + u_axis * (bore_px * 0.25);
        p.text(
            lbl_pos,
            Align2::CENTER_CENTER,
            format!("{}\n{:.1} bar", label, cyl.pressure_bar),
            FontId::monospace(11.0),
            Color32::from_rgb(215, 225, 235),
        );
    }

    // Shared Crankpin & Crankshaft Main Journal
    p.line_segment([crank_center, pin_pos], Stroke::new(12.0_f32, Color32::from_rgb(110, 120, 138)));
    p.circle_filled(pin_pos, 7.5, Color32::from_rgb(245, 185, 65));
    p.circle_filled(crank_center, 9.0, Color32::from_rgb(200, 208, 220));
}

fn paint_bank_cutaway(p: &egui::Painter, rect: Rect, snap: &TelemetrySnapshot) {
    // Show Cylinders 1..5 side-by-side along the Left Bank
    let pad = rect.shrink(24.0);
    let col_w = pad.width() / 5.0;
    let bore_px = (col_w * 0.56).min(pad.height() * 0.28);
    let crank_y = pad.bottom() - pad.height() * 0.20;
    let r_px = pad.height() * 0.11;
    let l_px = pad.height() * 0.34;

    // Crankshaft centerline
    p.line_segment(
        [Pos2::new(pad.left() + 18.0, crank_y), Pos2::new(pad.right() - 18.0, crank_y)],
        Stroke::new(6.0_f32, Color32::from_rgb(75, 82, 95)),
    );

    for i in 0..5 {
        let cyl = &snap.cylinders[i];
        let cx = pad.left() + (i as f32 + 0.5) * col_w;
        let theta = cyl.local_angle_rad as f32;

        // Crankpin relative to this cylinder's axis
        let pin_pos = Pos2::new(cx + theta.sin() * r_px, crank_y - theta.cos() * r_px);
        let wrist_y = pin_pos.y - (l_px * l_px - (pin_pos.x - cx).powi(2)).max(1.0).sqrt();
        let wrist_pos = Pos2::new(cx, wrist_y);

        let head_y = crank_y - r_px - l_px - bore_px * 0.30;
        let cyl_bot_y = crank_y - l_px + r_px + bore_px * 0.15;

        let sleeve = Rect::from_min_max(
            Pos2::new(cx - bore_px * 0.52, head_y),
            Pos2::new(cx + bore_px * 0.52, cyl_bot_y),
        );
        p.rect_filled(sleeve, 2.0, Color32::from_rgb(24, 28, 35));
        p.rect_stroke(sleeve, 2.0, Stroke::new(2.0_f32, Color32::from_rgb(78, 86, 102)));

        let crown_y = wrist_y - bore_px * 0.18;
        if crown_y > head_y {
            let gas_rect = Rect::from_min_max(
                Pos2::new(cx - bore_px * 0.48, head_y + 2.0),
                Pos2::new(cx + bore_px * 0.48, crown_y),
            );
            p.rect_filled(gas_rect, 0.0, gas_color(cyl));
        }

        // Intake & Exhaust valves
        let in_y = head_y + (cyl.intake_lift_norm as f32) * (bore_px * 0.14);
        let ex_y = head_y + (cyl.exhaust_lift_norm as f32) * (bore_px * 0.14);
        p.line_segment(
            [Pos2::new(cx - bore_px * 0.34, in_y), Pos2::new(cx - bore_px * 0.10, in_y)],
            Stroke::new(3.5_f32, Color32::from_rgb(130, 210, 255)),
        );
        p.line_segment(
            [Pos2::new(cx + bore_px * 0.10, ex_y), Pos2::new(cx + bore_px * 0.34, ex_y)],
            Stroke::new(3.5_f32, Color32::from_rgb(255, 145, 95)),
        );

        if cyl.spark_firing {
            p.circle_filled(Pos2::new(cx, head_y + 4.0), 6.0, Color32::from_rgb(120, 240, 255));
        }

        // Piston
        let piston_rect = Rect::from_min_max(
            Pos2::new(cx - bore_px * 0.46, crown_y),
            Pos2::new(cx + bore_px * 0.46, wrist_y + bore_px * 0.16),
        );
        p.rect_filled(piston_rect, 2.0, Color32::from_rgb(175, 185, 198));

        // Conrod & crankpin
        p.line_segment([Pos2::new(cx, crank_y), pin_pos], Stroke::new(8.0_f32, Color32::from_rgb(105, 115, 130)));
        p.line_segment([pin_pos, wrist_pos], Stroke::new(6.5_f32, Color32::from_rgb(145, 155, 170)));
        p.circle_filled(pin_pos, 5.5, Color32::from_rgb(245, 185, 65));
        p.circle_filled(wrist_pos, 3.8, Color32::WHITE);

        p.text(
            Pos2::new(cx, head_y - 16.0),
            Align2::CENTER_CENTER,
            format!("CYL {}\n{:.1} bar", i + 1, cyl.pressure_bar),
            FontId::monospace(10.5),
            Color32::from_rgb(215, 225, 235),
        );
    }
}

fn draw_firing_strip(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    ui.horizontal(|ui| {
        let avail_w = ui.available_width();
        let card_w = ((avail_w - 45.0) / NUM_CYL as f32).max(46.0);
        for &cyl_idx in &FIRING_ORDER {
            let cyl = &snap.cylinders[cyl_idx];
            let (tag, col) = match cyl.phase {
                StrokePhase::Power => ("PWR", Color32::from_rgb(255, 135, 50)),
                StrokePhase::Exhaust => ("EXH", Color32::from_rgb(215, 70, 65)),
                StrokePhase::Intake => ("INT", Color32::from_rgb(65, 175, 245)),
                StrokePhase::Compression => ("CMP", Color32::from_rgb(235, 190, 65)),
            };
            let (rect, _) = ui.allocate_exact_size(Vec2::new(card_w, 68.0), Sense::hover());
            let p = ui.painter_at(rect);
            p.rect_filled(rect, 5.0, Color32::from_rgb(22, 26, 33));
            p.rect_stroke(
                rect,
                5.0,
                Stroke::new(
                    if cyl.spark_firing { 1.8_f32 } else { 1.0_f32 },
                    if cyl.spark_firing {
                        Color32::from_rgb(120, 240, 255)
                    } else {
                        Color32::from_rgb(45, 52, 65)
                    },
                ),
            );

            // Pressure fill bar at bottom
            let p_frac = (cyl.pressure_bar as f32 / 90.0).clamp(0.02, 1.0);
            let bar_h = p_frac * (rect.height() - 26.0);
            let bar_rect = Rect::from_min_max(
                Pos2::new(rect.left() + 5.0, rect.bottom() - 5.0 - bar_h),
                Pos2::new(rect.right() - 5.0, rect.bottom() - 5.0),
            );
            p.rect_filled(bar_rect, Rounding::same(2.0), col.linear_multiply(0.75));

            p.text(
                Pos2::new(rect.center().x, rect.top() + 8.0),
                Align2::CENTER_CENTER,
                format!("#{} {}", cyl_idx + 1, tag),
                FontId::monospace(10.0),
                Color32::WHITE,
            );
            p.text(
                Pos2::new(rect.center().x, rect.bottom() - 13.0),
                Align2::CENTER_CENTER,
                format!("{:.0}b", cyl.pressure_bar),
                FontId::monospace(9.5),
                Color32::from_rgb(225, 230, 240),
            );
        }
    });
}
