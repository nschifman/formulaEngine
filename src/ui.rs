use std::f32::consts::PI;
use std::sync::{Arc, Mutex};

use eframe::egui::{
    self, Align2, Color32, CursorIcon, FontId, Pos2, Rect, RichText, Rounding, Sense, Stroke, Vec2,
};

use crate::sim::{
    BenchMode, BORE_M, CLEARANCE_VOL_M3, CONROD_M, CRANK_RADIUS_M, DISP_VOL_M3, DYNO_BINS,
    DYNO_MAX_RPM, DYNO_MIN_RPM, PV_POINTS, SCOPE_LEN, SharedControls, STROKE_M, StrokePhase,
    TelemetrySnapshot, V_ANGLE_DEG,
};

// Pure Obsidian Black Industrial Palette (single open-space aesthetic)
const BG_CANVAS: Color32 = Color32::from_rgb(5, 6, 8);
#[inline]
fn bg_tint() -> Color32 {
    Color32::from_rgba_unmultiplied(11, 13, 18, 215)
}
const BG_PILL: Color32 = Color32::from_rgb(15, 18, 24);
const BORDER_SUBTLE: Color32 = Color32::from_rgb(30, 35, 44);
const TEXT_PRIMARY: Color32 = Color32::from_rgb(228, 232, 240);
const TEXT_MUTED: Color32 = Color32::from_rgb(122, 130, 146);
const ACCENT_GREEN: Color32 = Color32::from_rgb(72, 188, 120);
const ACCENT_AMBER: Color32 = Color32::from_rgb(222, 158, 62);
const ACCENT_RED: Color32 = Color32::from_rgb(212, 74, 74);
const ACCENT_CYAN: Color32 = Color32::from_rgb(92, 180, 230);

#[derive(Clone, Copy, PartialEq, Eq)]
enum DiagnosticPlot {
    PvDiagram,
    Waveform,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum EnginePart {
    None,
    CylinderBlock,
    PneumaticHeads,
    SparkPlugs,
    CrankHubStarter,
    IntakeTrumpets,
    FlywheelHandCrank,
}

pub struct FormulaApp {
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
    diag_plot: DiagnosticPlot,
    show_ecu_drawer: bool,
    visuals_applied: bool,
    space_was_down: bool,

    // 3D Orbit Camera
    cam_yaw: f32,
    cam_pitch: f32,
    cam_zoom: f32,
    orbiting: bool,

    // Direct 3D / Slider Interaction State
    active_drag_part: EnginePart,
    holding_starter: bool,
    dragging_throttle_bar: bool,
    last_crank_pointer_angle: Option<f32>,
}

impl FormulaApp {
    pub fn new(
        controls: Arc<Mutex<SharedControls>>,
        telemetry: Arc<Mutex<TelemetrySnapshot>>,
    ) -> Self {
        Self {
            controls,
            telemetry,
            diag_plot: DiagnosticPlot::PvDiagram,
            show_ecu_drawer: false,
            visuals_applied: false,
            space_was_down: false,
            cam_yaw: -0.62,
            cam_pitch: 0.34,
            cam_zoom: 1.0,
            orbiting: false,
            active_drag_part: EnginePart::None,
            holding_starter: false,
            dragging_throttle_bar: false,
            last_crank_pointer_angle: None,
        }
    }

    fn apply_obsidian_style(&mut self, ctx: &egui::Context) {
        if self.visuals_applied {
            return;
        }
        self.visuals_applied = true;
        let mut style = (*ctx.style()).clone();
        style.visuals.window_fill = BG_CANVAS;
        style.visuals.panel_fill = BG_CANVAS;
        style.visuals.extreme_bg_color = BG_CANVAS;
        style.visuals.faint_bg_color = BG_PILL;
        style.visuals.widgets.noninteractive.bg_fill = Color32::TRANSPARENT;
        style.visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.inactive.bg_fill = BG_PILL;
        style.visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.hovered.bg_fill = Color32::from_rgb(26, 31, 40);
        style.visuals.widgets.hovered.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(56, 65, 82));
        style.visuals.widgets.active.bg_fill = Color32::from_rgb(36, 43, 56);
        style.visuals.selection.bg_fill = Color32::from_rgb(28, 66, 48);
        style.visuals.selection.stroke = Stroke::new(1.0_f32, ACCENT_GREEN);
        style.spacing.item_spacing = Vec2::new(6.0, 4.0);
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
                if !matches!(
                    ctrl.bench_mode,
                    BenchMode::TrackManualGearbox | BenchMode::MonzaHotlapAuto
                ) {
                    ctrl.bench_mode = BenchMode::TrackManualGearbox;
                    ctrl.requested_gear = 1;
                } else {
                    ctrl.requested_gear = (ctrl.requested_gear + 1).min(7);
                }
            }
            if i.key_pressed(egui::Key::ArrowLeft) || i.key_pressed(egui::Key::Z) {
                ctrl.requested_gear = ctrl.requested_gear.saturating_sub(1).max(1);
            }
            if i.key_pressed(egui::Key::Q) {
                ctrl.quick_start_trigger = true;
            }
            if i.key_pressed(egui::Key::R) {
                self.cam_yaw = -0.62;
                self.cam_pitch = 0.34;
                self.cam_zoom = 1.0;
            }
        });
    }
}

impl eframe::App for FormulaApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.apply_obsidian_style(ctx);
        ctx.request_repaint();

        let snap = { self.telemetry.lock().unwrap().clone() };
        let mut ctrl = { *self.controls.lock().unwrap() };

        self.handle_keyboard(ctx, &mut ctrl);

        // Single open obsidian canvas — zero child windows or split panel chrome
        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(BG_CANVAS))
            .show(ctx, |ui| {
                let full_rect = ui.max_rect();
                let painter = ui.painter_at(full_rect);

                // Subtle open-space floor grid reference (gives 3D orientation cue without boxes)
                let arena_center = Pos2::new(
                    full_rect.left() + full_rect.width() * 0.47,
                    full_rect.top() + full_rect.height() * 0.54,
                );
                let arena_scale_base = (full_rect.height() * 0.86).min(full_rect.width() * 0.58);

                // 1. CAMERA INPUT (Middle Mouse Click Drag to Orbit + Wheel Zoom)
                ctx.input(|i| {
                    let mmb_or_rmb = i.pointer.button_down(egui::PointerButton::Middle)
                        || i.pointer.button_down(egui::PointerButton::Secondary);
                    if mmb_or_rmb {
                        let delta = i.pointer.delta();
                        if self.orbiting || delta.length_sq() > 0.0 {
                            self.orbiting = true;
                            self.cam_yaw += delta.x * 0.0085;
                            self.cam_pitch = (self.cam_pitch + delta.y * 0.0070).clamp(-1.25, 1.35);
                        }
                    } else {
                        self.orbiting = false;
                    }

                    let scroll_y = i.raw_scroll_delta.y;
                    if scroll_y.abs() > 0.1 {
                        self.cam_zoom = (self.cam_zoom * (1.0 + scroll_y * 0.0012)).clamp(0.55, 2.35);
                    }
                });

                // 2. BUILD & DEPTH-SORT 3D V10 SCENE
                let cam = Camera3D::new(
                    arena_center,
                    arena_scale_base * self.cam_zoom,
                    self.cam_yaw,
                    self.cam_pitch,
                );
                let mut scene = Scene3D::default();
                let hits = build_v10_scene_3d(&cam, &mut scene, &snap, &ctrl);
                scene.render(&painter);

                // 3. HUD RECTS (explicitly excluded from 3D engine click-through)
                let tl_hud_rect = Rect::from_min_size(
                    Pos2::new(full_rect.left() + 24.0, full_rect.top() + 14.0),
                    Vec2::new(340.0, 56.0),
                );
                let throttle_bar_rect = Rect::from_min_size(
                    Pos2::new(full_rect.left() + 26.0, full_rect.top() + 104.0),
                    Vec2::new(26.0, (full_rect.height() - 390.0).clamp(210.0, 360.0)),
                );
                let throttle_hit_rect = throttle_bar_rect.expand2(Vec2::new(14.0, 10.0));
                let bottom_left_rect = Rect::from_min_max(
                    Pos2::new(full_rect.left() + 24.0, throttle_bar_rect.bottom() + 24.0),
                    Pos2::new(full_rect.left() + 275.0, full_rect.bottom() - 34.0),
                );
                let right_hud_rect = Rect::from_min_max(
                    Pos2::new(full_rect.right() - 306.0, full_rect.top() + 86.0),
                    Pos2::new(full_rect.right() - 18.0, full_rect.bottom() - 30.0),
                );

                // Clamp 3D callout tags inside the open center space so they never collide with HUDs
                let arena_tag_bounds = Rect::from_min_max(
                    Pos2::new(full_rect.left() + 260.0, full_rect.top() + 88.0),
                    Pos2::new(full_rect.right() - 325.0, full_rect.bottom() - 48.0),
                );
                let mut hits = hits;
                hits.clamp_tags(arena_tag_bounds);

                // LEFT-CLICK 3D ENGINE INTERACTION LAYER
                let engine_resp = ui.interact(
                    full_rect,
                    ui.id().with("engine_3d_space"),
                    Sense::click_and_drag(),
                );
                let pointer_pos = ctx.input(|i| i.pointer.interact_pos());
                let hover_pos = ctx.input(|i| i.pointer.hover_pos());
                let pos_over_hud = |p: Pos2| {
                    tl_hud_rect.contains(p)
                        || throttle_hit_rect.contains(p)
                        || bottom_left_rect.contains(p)
                        || right_hud_rect.contains(p)
                };
                let hovered_part = hover_pos
                    .filter(|p| !pos_over_hud(*p))
                    .map(|p| hits.hit_test(p))
                    .unwrap_or(EnginePart::None);

                if engine_resp.drag_started_by(egui::PointerButton::Primary) {
                    if let Some(pos) = pointer_pos.filter(|p| !pos_over_hud(*p)) {
                        let part = hits.hit_test(pos);
                        self.active_drag_part = part;
                        if part == EnginePart::FlywheelHandCrank {
                            let d = pos - hits.front_pulley_pos;
                            self.last_crank_pointer_angle = Some(d.y.atan2(d.x));
                        } else if part == EnginePart::CrankHubStarter {
                            self.holding_starter = true;
                        }
                    }
                }
                if engine_resp.dragged_by(egui::PointerButton::Primary) {
                    match self.active_drag_part {
                        EnginePart::IntakeTrumpets => {
                            let dy = engine_resp.drag_delta().y as f64;
                            ctrl.throttle = (ctrl.throttle - dy / 160.0).clamp(0.0, 1.0);
                        }
                        EnginePart::FlywheelHandCrank => {
                            if let Some(pos) = pointer_pos {
                                let d = pos - hits.front_pulley_pos;
                                let ang = d.y.atan2(d.x);
                                if let Some(prev) = self.last_crank_pointer_angle {
                                    let mut delta = ang - prev;
                                    if delta > PI {
                                        delta -= 2.0 * PI;
                                    } else if delta < -PI {
                                        delta += 2.0 * PI;
                                    }
                                    ctrl.manual_crank_delta_rad += delta as f64;
                                }
                                self.last_crank_pointer_angle = Some(ang);
                            }
                        }
                        _ => {}
                    }
                }
                if !ctx.input(|i| i.pointer.primary_down()) {
                    self.active_drag_part = EnginePart::None;
                    self.holding_starter = false;
                    self.last_crank_pointer_angle = None;
                }

                if engine_resp.clicked() {
                    if let Some(pos) = pointer_pos.filter(|p| !pos_over_hud(*p)) {
                        match hits.hit_test(pos) {
                            EnginePart::CylinderBlock => ctrl.set_preheat(!ctrl.preheat_on()),
                            EnginePart::PneumaticHeads => ctrl.toggle_pneumatics(),
                            EnginePart::SparkPlugs => ctrl.toggle_arm(),
                            _ => {}
                        }
                    }
                }

                // Holding primary pointer button down over Rear Starter engages Air Starter
                let mut starter_down = self.holding_starter;
                if engine_resp.is_pointer_button_down_on()
                    && ctx.input(|i| i.pointer.primary_down())
                {
                    if let Some(pos) = pointer_pos.filter(|p| !pos_over_hud(*p)) {
                        if hits.hit_test(pos) == EnginePart::CrankHubStarter {
                            self.holding_starter = true;
                            starter_down = true;
                        }
                    }
                }

                // Draw 3D Callout Tags directly anchored to the 3D engine parts
                let active_highlight = if self.active_drag_part != EnginePart::None {
                    self.active_drag_part
                } else {
                    hovered_part
                };
                draw_3d_callout_tags(&painter, &hits, &snap, &ctrl, active_highlight, starter_down);

                if self.orbiting {
                    ctx.set_cursor_icon(CursorIcon::Grabbing);
                } else if active_highlight != EnginePart::None {
                    let show_grab = matches!(
                        active_highlight,
                        EnginePart::IntakeTrumpets | EnginePart::FlywheelHandCrank
                    );
                    ctx.set_cursor_icon(if show_grab {
                        CursorIcon::Grab
                    } else {
                        CursorIcon::PointingHand
                    });
                }

                // 4. TOP OPEN-SPACE TELEMETRY & SHIFT LIGHTS
                draw_top_hud(ui, &painter, full_rect, tl_hud_rect, &snap, &mut ctrl, &mut self.cam_yaw, &mut self.cam_pitch, &mut self.cam_zoom);

                // 5. LEFT-SIDE VERTICAL THROTTLE SLIDER + KEYBIND MARKINGS
                draw_vertical_throttle_slider(
                    ui,
                    &painter,
                    throttle_bar_rect,
                    throttle_hit_rect,
                    &snap,
                    &mut ctrl,
                    &mut self.dragging_throttle_bar,
                );

                // 6. BOTTOM-LEFT COMPACT STARTUP FLOW + TEST MODE PILLS
                let pill_starter =
                    draw_bottom_left_hud(ui, bottom_left_rect, &snap, &mut ctrl, &mut self.show_ecu_drawer);
                ctrl.starter_active = starter_down || pill_starter;

                // 7. RIGHT-SIDE OPEN-SPACE DIAGNOSTICS (DYNO + P-V/SCOPE + VITALS)
                draw_right_hud(ui, right_hud_rect, &snap, &mut ctrl, &mut self.diag_plot);

                // 8. BOTTOM-CENTER SUBTLE INTERACTION FOOTER
                let footer_hint = match active_highlight {
                    EnginePart::CylinderBlock => {
                        "3D Cylinder Block Pre-Heater — Click to toggle Pre-Heat (22°C -> 60°C)"
                    }
                    EnginePart::PneumaticHeads => {
                        "3D N2 Pneumatic Bottle & Heads — Click to pressurize valve springs (165 bar)"
                    }
                    EnginePart::SparkPlugs => {
                        "3D V-Valley Fuel Rail & Coils — Click to arm Fuel & Ignition"
                    }
                    EnginePart::CrankHubStarter => {
                        "3D Rear Air Starter Drive — Click & Hold to crank the V10"
                    }
                    EnginePart::IntakeTrumpets => {
                        "3D Variable Intake Trumpets — Drag Up/Down to open throttle butterflies"
                    }
                    EnginePart::FlywheelHandCrank => {
                        if snap.rpm < 60.0 {
                            "3D Front Crank Pulley — Drag around center to bar engine over by hand"
                        } else {
                            "3D Front Crank Pulley — Rotating at engine speed"
                        }
                    }
                    EnginePart::None => {
                        "Middle Mouse Drag: Orbit 3D V10  •  Scroll: Zoom  •  Click 3D Parts (1->2->3->4) to Start  •  [R] Reset View"
                    }
                };
                painter.text(
                    Pos2::new(arena_center.x, full_rect.bottom() - 12.0),
                    Align2::CENTER_BOTTOM,
                    footer_hint,
                    FontId::monospace(10.0),
                    TEXT_MUTED,
                );
            });

        *self.controls.lock().unwrap() = ctrl;
    }
}

// ============================================================================
// 3D CAMERA & DEPTH-SORTED VECTOR RENDERER
// ============================================================================

struct Camera3D {
    origin_2d: Pos2,
    scale_px: f32,
    cam_dist: f32,
    right: [f32; 3],
    up: [f32; 3],
    fwd: [f32; 3],
    target: [f32; 3],
}

impl Camera3D {
    fn new(origin_2d: Pos2, scale_px: f32, yaw: f32, pitch: f32) -> Self {
        let cy = yaw.cos();
        let sy = yaw.sin();
        let cp = pitch.cos();
        let sp = pitch.sin();

        // Forward vector looking from camera toward target
        let fwd = [-sy * cp, -sp, cy * cp];
        let right = [cy, 0.0, sy];
        // up = fwd x right (screen +Y points down, so negate world Y sign when mapping to Pos2)
        let up = [
            fwd[1] * right[2] - fwd[2] * right[1],
            fwd[2] * right[0] - fwd[0] * right[2],
            fwd[0] * right[1] - fwd[1] * right[0],
        ];

        Self {
            origin_2d,
            scale_px,
            cam_dist: 1.55,
            right,
            up,
            fwd,
            target: [0.0, 0.14, 0.0],
        }
    }

    #[inline]
    fn project(&self, p: [f32; 3]) -> Option<(Pos2, f32, f32)> {
        let rel = [
            p[0] - self.target[0],
            p[1] - self.target[1],
            p[2] - self.target[2],
        ];
        let x_cam = rel[0] * self.right[0] + rel[1] * self.right[1] + rel[2] * self.right[2];
        let y_cam = rel[0] * self.up[0] + rel[1] * self.up[1] + rel[2] * self.up[2];
        let z_cam = self.cam_dist + (rel[0] * self.fwd[0] + rel[1] * self.fwd[1] + rel[2] * self.fwd[2]);
        if z_cam < 0.25 {
            return None;
        }
        let persp = self.scale_px / z_cam;
        let sx = self.origin_2d.x + x_cam * persp;
        let sy = self.origin_2d.y - y_cam * persp;
        Some((Pos2::new(sx, sy), z_cam, persp))
    }
}

#[inline]
fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn mul3(a: [f32; 3], s: f32) -> [f32; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn norm3(a: [f32; 3]) -> [f32; 3] {
    let l = dot3(a, a).sqrt().max(1e-6);
    [a[0] / l, a[1] / l, a[2] / l]
}

#[inline]
fn shade_color(base: Color32, normal: [f32; 3]) -> Color32 {
    let l = norm3([-0.35, 0.85, -0.40]);
    let n = norm3(normal);
    let ndotl = dot3(n, l).abs();
    let gain = 0.56 + 0.44 * ndotl;
    Color32::from_rgba_unmultiplied(
        ((base.r() as f32 * gain).min(255.0)) as u8,
        ((base.g() as f32 * gain).min(255.0)) as u8,
        ((base.b() as f32 * gain).min(255.0)) as u8,
        base.a(),
    )
}

enum Prim3D {
    Poly {
        pts: Vec<Pos2>,
        fill: Color32,
        stroke: Stroke,
    },
    Line {
        a: Pos2,
        b: Pos2,
        stroke: Stroke,
    },
    Circle {
        c: Pos2,
        r: f32,
        fill: Color32,
        stroke: Stroke,
    },
}

struct DrawItem3D {
    depth: f32,
    prim: Prim3D,
}

#[derive(Default)]
struct Scene3D {
    items: Vec<DrawItem3D>,
}

impl Scene3D {
    fn push_poly(&mut self, cam: &Camera3D, world_pts: &[[f32; 3]], fill: Color32, stroke: Stroke, shade: bool) {
        let mut pts = Vec::with_capacity(world_pts.len());
        let mut depth_sum = 0.0_f32;
        for &wp in world_pts {
            if let Some((p2, z, _)) = cam.project(wp) {
                pts.push(p2);
                depth_sum += z;
            } else {
                return;
            }
        }
        let final_fill = if shade && world_pts.len() >= 3 {
            let e1 = sub3(world_pts[1], world_pts[0]);
            let e2 = sub3(world_pts[2], world_pts[0]);
            let n = [
                e1[1] * e2[2] - e1[2] * e2[1],
                e1[2] * e2[0] - e1[0] * e2[2],
                e1[0] * e2[1] - e1[1] * e2[0],
            ];
            shade_color(fill, n)
        } else {
            fill
        };
        self.items.push(DrawItem3D {
            depth: depth_sum / world_pts.len() as f32,
            prim: Prim3D::Poly {
                pts,
                fill: final_fill,
                stroke,
            },
        });
    }

    fn push_rod(&mut self, cam: &Camera3D, a_w: [f32; 3], b_w: [f32; 3], radius_m: f32, col: Color32) {
        if let (Some((pa, za, s_a)), Some((pb, zb, s_b))) = (cam.project(a_w), cam.project(b_w)) {
            let px_w = (radius_m * 2.0 * 0.5 * (s_a + s_b)).clamp(1.2, 34.0);
            self.items.push(DrawItem3D {
                depth: 0.5 * (za + zb),
                prim: Prim3D::Line {
                    a: pa,
                    b: pb,
                    stroke: Stroke::new(px_w, col),
                },
            });
        }
    }

    fn push_line(&mut self, cam: &Camera3D, a_w: [f32; 3], b_w: [f32; 3], stroke: Stroke) {
        if let (Some((pa, za, _)), Some((pb, zb, _))) = (cam.project(a_w), cam.project(b_w)) {
            self.items.push(DrawItem3D {
                depth: 0.5 * (za + zb),
                prim: Prim3D::Line { a: pa, b: pb, stroke },
            });
        }
    }

    fn push_sphere(&mut self, cam: &Camera3D, c_w: [f32; 3], radius_m: f32, fill: Color32, stroke: Stroke) {
        if let Some((pc, zc, s)) = cam.project(c_w) {
            self.items.push(DrawItem3D {
                depth: zc,
                prim: Prim3D::Circle {
                    c: pc,
                    r: (radius_m * s).clamp(1.5, 42.0),
                    fill,
                    stroke,
                },
            });
        }
    }

    /// Push a 3D circular ring/disc oriented perpendicular to axis `axis_u, axis_v`
    fn push_ring_3d(
        &mut self,
        cam: &Camera3D,
        center: [f32; 3],
        u_dir: [f32; 3],
        v_dir: [f32; 3],
        radius: f32,
        segs: usize,
        fill: Color32,
        stroke: Stroke,
    ) {
        let mut pts = Vec::with_capacity(segs);
        for k in 0..segs {
            let a = (k as f32 / segs as f32) * 2.0 * PI;
            let wp = add3(center, add3(mul3(u_dir, a.cos() * radius), mul3(v_dir, a.sin() * radius)));
            pts.push(wp);
        }
        self.push_poly(cam, &pts, fill, stroke, false);
    }

    fn push_cylinder_shell(
        &mut self,
        cam: &Camera3D,
        p0: [f32; 3],
        p1: [f32; 3],
        u_dir: [f32; 3],
        v_dir: [f32; 3],
        r0: f32,
        r1: f32,
        segs: usize,
        fill: Color32,
        stroke: Stroke,
        shade: bool,
    ) {
        for k in 0..segs {
            let a0 = (k as f32 / segs as f32) * 2.0 * PI;
            let a1 = ((k + 1) as f32 / segs as f32) * 2.0 * PI;
            let n0 = add3(mul3(u_dir, a0.cos()), mul3(v_dir, a0.sin()));
            let n1 = add3(mul3(u_dir, a1.cos()), mul3(v_dir, a1.sin()));
            let q = [
                add3(p0, mul3(n0, r0)),
                add3(p1, mul3(n0, r1)),
                add3(p1, mul3(n1, r1)),
                add3(p0, mul3(n1, r0)),
            ];
            self.push_poly(cam, &q, fill, stroke, shade);
        }
    }

    fn render(&mut self, p: &egui::Painter) {
        self.items
            .sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap_or(std::cmp::Ordering::Equal));
        for item in &self.items {
            match &item.prim {
                Prim3D::Poly { pts, fill, stroke } => {
                    p.add(egui::Shape::convex_polygon(pts.clone(), *fill, *stroke));
                }
                Prim3D::Line { a, b, stroke } => {
                    p.line_segment([*a, *b], *stroke);
                }
                Prim3D::Circle { c, r, fill, stroke } => {
                    if *fill != Color32::TRANSPARENT {
                        p.circle_filled(*c, *r, *fill);
                    }
                    if stroke.width > 0.0 && stroke.color != Color32::TRANSPARENT {
                        p.circle_stroke(*c, *r, *stroke);
                    }
                }
            }
        }
    }
}

struct HitTargets3D {
    preheat_pts: Vec<Pos2>,
    n2_pts: Vec<Pos2>,
    rail_pts: Vec<Pos2>,
    starter_pos: Pos2,
    front_pulley_pos: Pos2,
    trumpet_pts: Vec<Pos2>,
    tag_preheat: Rect,
    tag_n2: Rect,
    tag_rail: Rect,
    tag_starter: Rect,
}

impl HitTargets3D {
    fn clamp_tags(&mut self, bounds: Rect) {
        let clamp_one = |r: &mut Rect| {
            let sz = r.size();
            let half = sz * 0.5;
            let cx = r.center().x.clamp(bounds.left() + half.x, (bounds.right() - half.x).max(bounds.left() + half.x));
            let cy = r.center().y.clamp(bounds.top() + half.y, (bounds.bottom() - half.y).max(bounds.top() + half.y));
            *r = Rect::from_center_size(Pos2::new(cx, cy), sz);
        };
        clamp_one(&mut self.tag_preheat);
        clamp_one(&mut self.tag_n2);
        clamp_one(&mut self.tag_rail);
        clamp_one(&mut self.tag_starter);
    }

    fn hit_test(&self, pos: Pos2) -> EnginePart {
        if self.tag_starter.contains(pos) || (pos - self.starter_pos).length() < 34.0 {
            return EnginePart::CrankHubStarter;
        }
        if self.tag_n2.contains(pos) {
            return EnginePart::PneumaticHeads;
        }
        if self.tag_rail.contains(pos) {
            return EnginePart::SparkPlugs;
        }
        if self.tag_preheat.contains(pos) {
            return EnginePart::CylinderBlock;
        }
        if (pos - self.front_pulley_pos).length() < 40.0 {
            return EnginePart::FlywheelHandCrank;
        }
        for &p in &self.n2_pts {
            if (pos - p).length() < 26.0 {
                return EnginePart::PneumaticHeads;
            }
        }
        for &p in &self.rail_pts {
            if (pos - p).length() < 22.0 {
                return EnginePart::SparkPlugs;
            }
        }
        for &p in &self.trumpet_pts {
            if (pos - p).length() < 28.0 {
                return EnginePart::IntakeTrumpets;
            }
        }
        for &p in &self.preheat_pts {
            if (pos - p).length() < 32.0 {
                return EnginePart::CylinderBlock;
            }
        }
        EnginePart::None
    }
}

fn build_v10_scene_3d(
    cam: &Camera3D,
    scene: &mut Scene3D,
    snap: &TelemetrySnapshot,
    ctrl: &SharedControls,
) -> HitTargets3D {
    let bore = BORE_M as f32;
    let r_crank = CRANK_RADIUS_M as f32;
    let l_rod = CONROD_M as f32;
    let stroke = STROKE_M as f32;
    let half_v = (V_ANGLE_DEG as f32 * 0.5).to_radians();
    let throw_spacing = 0.112_f32;
    let z_front = -2.0 * throw_spacing - 0.055;
    let z_rear = 2.0 * throw_spacing + 0.055;

    let tdc_wrist = r_crank + l_rod;
    let piston_h = bore * 0.30;
    let crown_offset = piston_h * 0.45;
    let head_dist = tdc_wrist + crown_offset + stroke * 0.14;
    let cyl_bot_dist = l_rod - r_crank - piston_h * 0.28;
    let mid_dist = 0.5 * (head_dist + cyl_bot_dist);
    let trumpet_len_m = (snap.trumpet_length_mm as f32) * 0.001;

    // 0. Subtle 3D Engine Stand / Floor Reference Crosshairs
    let floor_y = -0.075_f32;
    for g in -2..=2 {
        let off = g as f32 * 0.14;
        scene.push_line(
            cam,
            [off, floor_y, z_front - 0.08],
            [off, floor_y, z_rear + 0.08],
            Stroke::new(1.0_f32, Color32::from_rgb(16, 19, 25)),
        );
        scene.push_line(
            cam,
            [-0.28, floor_y, off * 1.15],
            [0.28, floor_y, off * 1.15],
            Stroke::new(1.0_f32, Color32::from_rgb(16, 19, 25)),
        );
    }

    // 1. Central Main Bearing Bulkheads & Sump Bedplate
    for b in 0..=5 {
        let zb = -2.5 * throw_spacing + (b as f32) * throw_spacing;
        scene.push_ring_3d(
            cam,
            [0.0, 0.0, zb],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            r_crank * 2.15,
            10,
            Color32::from_rgba_unmultiplied(18, 21, 28, 175),
            Stroke::new(1.0_f32, Color32::from_rgb(38, 44, 56)),
        );
    }

    // 2. Thermal Block Pre-Heater Manifolds (Left & Right lower crankcase heat rails)
    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
    let preheat_col = if min_temp >= 60.0 {
        Color32::from_rgb(56, 138, 92)
    } else if ctrl.preheat_on() {
        Color32::from_rgb(205, 132, 46)
    } else {
        Color32::from_rgb(66, 74, 88)
    };
    let mut preheat_pts = Vec::new();
    for &sign_x in &[-1.0_f32, 1.0_f32] {
        let hx = sign_x * 0.145;
        let hy = 0.055;
        scene.push_rod(cam, [hx, hy, z_front], [hx, hy, z_rear], 0.011, preheat_col);
        for &z_pt in &[z_front + 0.06, 0.0, z_rear - 0.06] {
            if let Some((p2, _, _)) = cam.project([hx, hy, z_pt]) {
                preheat_pts.push(p2);
            }
        }
    }

    // 3. Crankshaft Main Axis, 5 Throws, Counterweights, Pistons, Rods, Cylinders, Valves, Trumpets, Exhausts
    scene.push_rod(
        cam,
        [0.0, 0.0, z_front - 0.055],
        [0.0, 0.0, z_rear + 0.055],
        0.016,
        Color32::from_rgb(92, 102, 118),
    );

    let ex_col = header_glow_color(snap.egt_c);
    let mut trumpet_pts = Vec::new();
    let mut rail_pts = Vec::new();
    let mut n2_pts = Vec::new();

    // Left & Right 5-into-1 Collector tips
    let col_tip_l = [-0.285_f32, 0.095_f32, z_rear + 0.04];
    let col_tip_r = [0.285_f32, 0.095_f32, z_rear + 0.04];
    scene.push_cylinder_shell(
        cam,
        add3(col_tip_l, [0.0, 0.0, -0.05]),
        add3(col_tip_l, [0.0, 0.0, 0.14]),
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.025,
        0.028,
        8,
        ex_col,
        Stroke::new(1.0_f32, Color32::from_rgb(45, 36, 34)),
        true,
    );
    scene.push_cylinder_shell(
        cam,
        add3(col_tip_r, [0.0, 0.0, -0.05]),
        add3(col_tip_r, [0.0, 0.0, 0.14]),
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.025,
        0.028,
        8,
        ex_col,
        Stroke::new(1.0_f32, Color32::from_rgb(45, 36, 34)),
        true,
    );

    let mut bank_head_top_l = [0.0_f32; 3];
    let mut bank_head_top_r = [0.0_f32; 3];

    for throw_idx in 0..5 {
        let zt = (throw_idx as f32 - 2.0) * throw_spacing;
        let cyl_l_idx = throw_idx;
        let cyl_r_idx = throw_idx + 5;

        // Shared crankpin angle so both Left and Right hit TDC at local_angle_rad == 0
        let pin_angle = (snap.cylinders[cyl_l_idx].local_angle_rad as f32) - half_v;
        let pin_pos = [pin_angle.sin() * r_crank, pin_angle.cos() * r_crank, zt];
        let cw_pos = [-pin_angle.sin() * (r_crank * 0.9), -pin_angle.cos() * (r_crank * 0.9), zt];

        // Crank web & counterweight
        scene.push_rod(cam, [0.0, 0.0, zt], pin_pos, 0.011, Color32::from_rgb(138, 148, 165));
        scene.push_sphere(
            cam,
            cw_pos,
            r_crank * 1.12,
            Color32::from_rgb(56, 62, 74),
            Stroke::new(1.0_f32, Color32::from_rgb(88, 96, 112)),
        );
        scene.push_sphere(cam, pin_pos, 0.0085, ACCENT_AMBER, Stroke::NONE);

        for &(cyl_idx, bank_ang, col_tip) in &[
            (cyl_l_idx, -half_v, col_tip_l),
            (cyl_r_idx, half_v, col_tip_r),
        ] {
            let cyl = &snap.cylinders[cyl_idx];
            // Offset Left slightly forward and Right slightly aft within each shared throw bay
            let z_cyl = zt + if bank_ang < 0.0 { -0.012 } else { 0.012 };
            let u_axis = [bank_ang.sin(), bank_ang.cos(), 0.0];
            let u_perp = [bank_ang.cos(), -bank_ang.sin(), 0.0];
            let u_z = [0.0, 0.0, 1.0];

            // Kinematic slider-crank wrist pin along cylinder axis in XY
            let d = [pin_pos[0], pin_pos[1], 0.0];
            let proj = dot3(d, u_axis);
            let perp = dot3(d, u_perp);
            let wrist_dist = proj + (l_rod * l_rod - perp * perp).max(0.0001).sqrt();
            let wrist_pos = add3([0.0, 0.0, z_cyl], mul3(u_axis, wrist_dist));

            // 3D Titanium Conrod
            scene.push_rod(
                cam,
                [pin_pos[0], pin_pos[1], z_cyl],
                wrist_pos,
                0.0075,
                Color32::from_rgb(142, 152, 168),
            );

            // 3D Billet Piston Crown + Skirt
            let crown_dist = wrist_dist + crown_offset;
            let skirt_dist = wrist_dist - piston_h * 0.45;
            scene.push_cylinder_shell(
                cam,
                add3([0.0, 0.0, z_cyl], mul3(u_axis, skirt_dist)),
                add3([0.0, 0.0, z_cyl], mul3(u_axis, crown_dist)),
                u_perp,
                u_z,
                bore * 0.45,
                bore * 0.45,
                8,
                Color32::from_rgb(172, 180, 194),
                Stroke::new(0.8_f32, Color32::from_rgb(100, 108, 122)),
                true,
            );
            scene.push_ring_3d(
                cam,
                add3([0.0, 0.0, z_cyl], mul3(u_axis, crown_dist)),
                u_perp,
                u_z,
                bore * 0.45,
                8,
                Color32::from_rgb(198, 206, 218),
                Stroke::new(0.9_f32, Color32::from_rgb(125, 134, 148)),
            );

            // 3D Combustion Gas Chamber Volume above Piston Crown
            if head_dist > crown_dist + 0.001 {
                let g_col = gas_color(cyl);
                scene.push_cylinder_shell(
                    cam,
                    add3([0.0, 0.0, z_cyl], mul3(u_axis, crown_dist)),
                    add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist)),
                    u_perp,
                    u_z,
                    bore * 0.44,
                    bore * 0.44,
                    8,
                    g_col,
                    Stroke::NONE,
                    false,
                );
            }

            // 3D Translucent Cylinder Liner Sleeve (cutaway glass-metallic liner so piston/gas remain visible)
            let sleeve_col = if ctrl.preheat_on() && min_temp < 60.0 {
                Color32::from_rgb(145, 96, 40)
            } else {
                Color32::from_rgb(54, 62, 76)
            };
            scene.push_ring_3d(
                cam,
                add3([0.0, 0.0, z_cyl], mul3(u_axis, cyl_bot_dist)),
                u_perp,
                u_z,
                bore * 0.49,
                10,
                Color32::from_rgba_unmultiplied(18, 22, 29, 45),
                Stroke::new(1.0_f32, sleeve_col),
            );
            scene.push_ring_3d(
                cam,
                add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist)),
                u_perp,
                u_z,
                bore * 0.49,
                10,
                Color32::from_rgba_unmultiplied(24, 28, 36, 55),
                Stroke::new(1.2_f32, sleeve_col),
            );
            // Vertical cutaway liner struts (4 corners)
            for &sa in &[0.0_f32, PI * 0.5, PI, PI * 1.5] {
                let off = add3(mul3(u_perp, sa.cos() * bore * 0.49), mul3(u_z, sa.sin() * bore * 0.49));
                scene.push_line(
                    cam,
                    add3(add3([0.0, 0.0, z_cyl], mul3(u_axis, cyl_bot_dist)), off),
                    add3(add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist)), off),
                    Stroke::new(0.8_f32, Color32::from_rgba_unmultiplied(74, 84, 102, 130)),
                );
            }

            let head_c = add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist));
            let block_mid = add3([0.0, 0.0, z_cyl], mul3(u_axis, mid_dist));
            if let Some((p2, _, _)) = cam.project(block_mid) {
                preheat_pts.push(p2);
            }

            // Intake & Exhaust Valve Directions (V-inward vs V-outward)
            let in_sign = if bank_ang < 0.0 { 1.0 } else { -1.0 };
            let ex_sign = -in_sign;
            let in_port = add3(head_c, mul3(u_perp, in_sign * bore * 0.22));
            let ex_port = add3(head_c, mul3(u_perp, ex_sign * bore * 0.22));

            // 4 Valves per cylinder (front/rear valve pair on intake & exhaust side)
            let in_travel = (cyl.intake_lift_norm as f32) * (bore * 0.14);
            let ex_travel = (cyl.exhaust_lift_norm as f32) * (bore * 0.14);
            let valve_head_col_in = Color32::from_rgb(130, 200, 245);
            let valve_head_col_ex = Color32::from_rgb(240, 140, 95);
            for &vz in &[-bore * 0.16, bore * 0.16] {
                let seat_in = add3(add3(in_port, [0.0, 0.0, vz]), mul3(u_axis, -in_travel));
                let stem_in = add3(seat_in, mul3(u_axis, bore * 0.32));
                scene.push_rod(cam, seat_in, stem_in, 0.0024, valve_head_col_in);
                scene.push_sphere(cam, seat_in, 0.009, valve_head_col_in, Stroke::NONE);

                let seat_ex = add3(add3(ex_port, [0.0, 0.0, vz]), mul3(u_axis, -ex_travel));
                let stem_ex = add3(seat_ex, mul3(u_axis, bore * 0.32));
                scene.push_rod(cam, seat_ex, stem_ex, 0.0024, valve_head_col_ex);
                scene.push_sphere(cam, seat_ex, 0.008, valve_head_col_ex, Stroke::NONE);
            }

            // Pneumatic Valve Cover / Cam Box Rail
            let cover_c = add3(head_c, mul3(u_axis, bore * 0.34));
            let cover_col = if ctrl.n2_pressure_bar >= 100.0 {
                Color32::from_rgb(54, 102, 142)
            } else {
                Color32::from_rgb(44, 50, 62)
            };
            scene.push_sphere(cam, cover_c, 0.015, cover_col, Stroke::NONE);
            if let Some((p2, _, _)) = cam.project(cover_c) {
                n2_pts.push(p2);
            }
            if throw_idx == 2 {
                if bank_ang < 0.0 {
                    bank_head_top_l = cover_c;
                } else {
                    bank_head_top_r = cover_c;
                }
            }

            // Spark Plug + Ignition Flash
            if cyl.spark_firing {
                scene.push_sphere(cam, head_c, 0.016, Color32::from_rgb(130, 232, 255), Stroke::NONE);
                scene.push_sphere(cam, head_c, 0.008, Color32::WHITE, Stroke::NONE);
            }

            // Variable-Length Intake Stack & Trumpet pointing into the V-valley
            let runner_base = add3(in_port, mul3(u_axis, 0.015));
            let stack_dir = norm3(add3(mul3(u_axis, 0.92), mul3(u_perp, in_sign * 0.28)));
            let trumpet_top = add3(runner_base, mul3(stack_dir, 0.032 + trumpet_len_m));
            scene.push_cylinder_shell(
                cam,
                runner_base,
                trumpet_top,
                u_perp,
                u_z,
                bore * 0.21,
                bore * 0.26,
                8,
                Color32::from_rgb(44, 50, 60),
                Stroke::new(0.8_f32, Color32::from_rgb(84, 94, 110)),
                true,
            );
            scene.push_ring_3d(
                cam,
                trumpet_top,
                u_perp,
                u_z,
                bore * 0.27,
                10,
                Color32::from_rgb(28, 32, 40),
                Stroke::new(2.0_f32, Color32::from_rgb(135, 148, 166)),
            );
            // Brass Throttle Butterfly Plate inside each runner
            let tps_ang = (snap.effective_throttle as f32) * 1.48;
            let butterfly_c = add3(runner_base, mul3(stack_dir, 0.025));
            let plate_dir = add3(mul3(u_perp, tps_ang.cos()), mul3(stack_dir, tps_ang.sin()));
            scene.push_line(
                cam,
                sub3(butterfly_c, mul3(plate_dir, bore * 0.17)),
                add3(butterfly_c, mul3(plate_dir, bore * 0.17)),
                Stroke::new(2.6_f32, ACCENT_AMBER),
            );
            if cyl.fuel_injecting {
                scene.push_line(
                    cam,
                    add3(runner_base, mul3(stack_dir, 0.018)),
                    add3(in_port, mul3(u_axis, -0.012)),
                    Stroke::new(2.4_f32, Color32::from_rgba_unmultiplied(92, 180, 230, 175)),
                );
            }
            if let Some((p2, _, _)) = cam.project(trumpet_top) {
                trumpet_pts.push(p2);
            }

            // 5-into-1 Swept Inconel Exhaust Primary Tube
            let ex_bend1 = add3(add3(ex_port, mul3(u_axis, 0.022)), mul3(u_perp, ex_sign * 0.055));
            let ex_bend2 = [
                ex_sign * 0.255,
                0.150,
                z_cyl + 0.045,
            ];
            let col_entry = add3(col_tip, [0.0, 0.0, -0.04]);
            scene.push_rod(cam, ex_port, ex_bend1, 0.014, ex_col);
            scene.push_rod(cam, ex_bend1, ex_bend2, 0.014, ex_col);
            scene.push_rod(cam, ex_bend2, col_entry, 0.014, ex_col);
        }
    }

    // 4. Longitudinal Pneumatic Valve Cam Rails (Top of Left & Right Cylinder Heads)
    let pneu_rail_col = if ctrl.n2_pressure_bar >= 100.0 {
        Color32::from_rgb(66, 138, 190)
    } else {
        Color32::from_rgb(56, 64, 78)
    };
    for &bank_ang in &[-half_v, half_v] {
        let u_axis = [bank_ang.sin(), bank_ang.cos(), 0.0];
        let rail_off = mul3(u_axis, head_dist + bore * 0.34);
        let ra = add3([0.0, 0.0, z_front + 0.02], rail_off);
        let rb = add3([0.0, 0.0, z_rear - 0.02], rail_off);
        scene.push_rod(cam, ra, rb, 0.013, pneu_rail_col);
    }

    // 5. V-Valley Central Fuel Rail & Ignition Coil Pack Spine (Click Step 3!)
    let rail_col = if ctrl.armed() {
        ACCENT_GREEN
    } else {
        Color32::from_rgb(86, 94, 110)
    };
    let rail_y = head_dist * 0.94 + 0.036;
    scene.push_rod(
        cam,
        [0.0, rail_y, z_front + 0.03],
        [0.0, rail_y, z_rear - 0.03],
        0.012,
        rail_col,
    );
    for t in 0..5 {
        let zt = (t as f32 - 2.0) * throw_spacing;
        let cp = [0.0, rail_y, zt];
        scene.push_sphere(cam, cp, 0.013, rail_col, Stroke::new(1.0_f32, TEXT_PRIMARY));
        if let Some((p2, _, _)) = cam.project(cp) {
            rail_pts.push(p2);
        }
    }

    // 6. Side High-Pressure N2 Pneumatic Accumulator Bottle & Braided Feed Line (Click Step 2!)
    let n2_col = if ctrl.n2_pressure_bar >= 100.0 {
        ACCENT_CYAN
    } else {
        Color32::from_rgb(60, 72, 92)
    };
    let bot_a = [0.235_f32, 0.035_f32, -0.09_f32];
    let bot_b = [0.235_f32, 0.035_f32, 0.17_f32];
    scene.push_cylinder_shell(
        cam,
        bot_a,
        bot_b,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.036,
        0.036,
        10,
        n2_col,
        Stroke::new(1.0_f32, Color32::from_rgb(35, 50, 70)),
        true,
    );
    scene.push_sphere(cam, bot_a, 0.034, n2_col, Stroke::NONE);
    scene.push_sphere(cam, bot_b, 0.034, n2_col, Stroke::NONE);
    // Feed lines from N2 bottle up to both pneumatic cylinder heads
    let n2_mid = [0.235_f32, 0.035_f32, 0.04_f32];
    scene.push_line(cam, n2_mid, bank_head_top_r, Stroke::new(2.0_f32, n2_col));
    scene.push_line(cam, bank_head_top_r, bank_head_top_l, Stroke::new(1.6_f32, n2_col));
    for &bp in &[bot_a, n2_mid, bot_b] {
        if let Some((p2, _, _)) = cam.project(bp) {
            n2_pts.push(p2);
        }
    }

    // 7. Front Timing Pulley / Hand-Crank Damper (z = z_front - 0.04)
    let front_c = [0.0, 0.0, z_front - 0.04];
    let pulley_r = r_crank * 2.5;
    scene.push_ring_3d(
        cam,
        front_c,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        pulley_r,
        14,
        Color32::from_rgb(34, 39, 48),
        Stroke::new(1.6_f32, Color32::from_rgb(104, 114, 132)),
    );
    let ca = snap.crank_angle_rad as f32;
    scene.push_line(
        cam,
        front_c,
        [ca.sin() * pulley_r * 0.85, ca.cos() * pulley_r * 0.85, front_c[2]],
        Stroke::new(3.0_f32, ACCENT_AMBER),
    );
    scene.push_sphere(cam, front_c, 0.012, Color32::from_rgb(175, 185, 200), Stroke::NONE);
    let front_pulley_pos = cam
        .project(front_c)
        .map(|(p, _, _)| p)
        .unwrap_or(Pos2::ZERO);

    // 8. Rear Carbon Flywheel + External Pneumatic Air Starter Drive (Click & Hold Step 4!)
    let fw_c = [0.0, 0.0, z_rear + 0.04];
    let fw_r = r_crank * 3.8;
    scene.push_ring_3d(
        cam,
        fw_c,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        fw_r,
        18,
        Color32::from_rgb(24, 27, 34),
        Stroke::new(2.0_f32, Color32::from_rgb(86, 96, 114)),
    );
    scene.push_line(
        cam,
        [ca.sin() * (-fw_r * 0.9), ca.cos() * (-fw_r * 0.9), fw_c[2]],
        [ca.sin() * (fw_r * 0.9), ca.cos() * (fw_r * 0.9), fw_c[2]],
        Stroke::new(2.4_f32, Color32::from_rgb(122, 132, 150)),
    );

    let ready_to_crank = min_temp >= 60.0 && ctrl.n2_pressure_bar >= 100.0 && ctrl.armed() && !snap.is_running;
    let starter_col = if ctrl.starter_active {
        ACCENT_AMBER
    } else if ready_to_crank {
        ACCENT_GREEN
    } else {
        Color32::from_rgb(90, 98, 112)
    };
    let starter_a = [0.0, 0.0, z_rear + 0.055];
    let starter_b = [0.0, 0.0, z_rear + 0.175];
    scene.push_cylinder_shell(
        cam,
        starter_a,
        starter_b,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.032,
        0.036,
        10,
        starter_col,
        Stroke::new(1.2_f32, Color32::from_rgb(20, 24, 30)),
        true,
    );
    scene.push_ring_3d(
        cam,
        starter_b,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.036,
        10,
        starter_col,
        Stroke::new(1.2_f32, Color32::WHITE),
    );
    let starter_pos = cam
        .project(starter_b)
        .map(|(p, _, _)| p)
        .unwrap_or(Pos2::ZERO);

    // Project clean non-overlapping leader tag rects for the 4 3D startup subsystems
    let pre_anchor = cam
        .project([-0.155, 0.055, -0.06])
        .map(|(p, _, _)| p)
        .unwrap_or(Pos2::new(400.0, 500.0));
    let n2_anchor = cam
        .project([0.235, 0.035, 0.05])
        .map(|(p, _, _)| p)
        .unwrap_or(Pos2::new(720.0, 480.0));
    let rail_anchor = cam
        .project([0.0, rail_y + 0.04, -0.10])
        .map(|(p, _, _)| p)
        .unwrap_or(Pos2::new(560.0, 260.0));

    let tag_size = Vec2::new(136.0, 22.0);
    let tag_preheat = Rect::from_center_size(pre_anchor + Vec2::new(-52.0, 38.0), tag_size);
    let tag_n2 = Rect::from_center_size(n2_anchor + Vec2::new(62.0, 36.0), tag_size);
    let tag_rail = Rect::from_center_size(rail_anchor + Vec2::new(0.0, -56.0), tag_size);
    let tag_starter = Rect::from_center_size(starter_pos + Vec2::new(48.0, 32.0), tag_size);

    HitTargets3D {
        preheat_pts,
        n2_pts,
        rail_pts,
        starter_pos,
        front_pulley_pos,
        trumpet_pts,
        tag_preheat,
        tag_n2,
        tag_rail,
        tag_starter,
    }
}

fn draw_3d_callout_tags(
    p: &egui::Painter,
    hits: &HitTargets3D,
    snap: &TelemetrySnapshot,
    ctrl: &SharedControls,
    hovered: EnginePart,
    starter_down: bool,
) {
    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
    let warm_ok = min_temp >= 60.0;
    let pneu_ok = ctrl.n2_pressure_bar >= 100.0;
    let armed_ok = ctrl.armed();

    let next_step = if !warm_ok {
        1
    } else if !pneu_ok {
        2
    } else if !armed_ok {
        3
    } else if !snap.is_running {
        4
    } else {
        0
    };

    let draw_tag = |rect: Rect, anchor: Pos2, title: String, step_id: usize, done: bool, active: bool, hovered_me: bool| {
        let col = if active {
            ACCENT_AMBER
        } else if done {
            ACCENT_GREEN
        } else if next_step == step_id || hovered_me {
            ACCENT_AMBER
        } else {
            TEXT_MUTED
        };
        let border_w = if next_step == step_id || hovered_me || active {
            1.5_f32
        } else {
            0.9_f32
        };
        p.line_segment([anchor, rect.center()], Stroke::new(1.0_f32, col));
        p.circle_filled(anchor, 2.8, col);
        p.rect_filled(rect, 3.0, bg_tint());
        p.rect_stroke(rect, 3.0, Stroke::new(border_w, col));
        p.text(
            rect.center(),
            Align2::CENTER_CENTER,
            title,
            FontId::monospace(10.0),
            col,
        );
    };

    // Hide tags when running smoothly at high RPM unless hovered so view stays ultra clean
    let show_tags = !snap.is_running || hovered != EnginePart::None || starter_down;
    if !show_tags {
        return;
    }

    let pre_a = hits.preheat_pts.first().copied().unwrap_or(hits.tag_preheat.center());
    draw_tag(
        hits.tag_preheat,
        pre_a,
        if warm_ok {
            format!("1 PREHEAT {:.0}C", min_temp)
        } else if ctrl.preheat_on() {
            format!("1 HEATING {:.0}C", min_temp)
        } else {
            "1 CLICK PRE-HEAT".to_string()
        },
        1,
        warm_ok,
        ctrl.preheat_on() && !warm_ok,
        hovered == EnginePart::CylinderBlock,
    );

    let n2_a = hits.n2_pts.last().copied().unwrap_or(hits.tag_n2.center());
    draw_tag(
        hits.tag_n2,
        n2_a,
        if pneu_ok {
            format!("2 N2: {:.0} BAR", ctrl.n2_pressure_bar)
        } else {
            "2 CLICK N2 (0 BAR)".to_string()
        },
        2,
        pneu_ok,
        false,
        hovered == EnginePart::PneumaticHeads,
    );

    let rail_a = hits.rail_pts.first().copied().unwrap_or(hits.tag_rail.center());
    draw_tag(
        hits.tag_rail,
        rail_a,
        if armed_ok {
            "3 FUEL+IGN: ARMED".to_string()
        } else {
            "3 CLICK FUEL & IGN".to_string()
        },
        3,
        armed_ok,
        false,
        hovered == EnginePart::SparkPlugs,
    );

    draw_tag(
        hits.tag_starter,
        hits.starter_pos,
        if starter_down {
            "4 CRANKING...".to_string()
        } else if snap.is_running {
            "4 STARTER READY".to_string()
        } else {
            "4 HOLD STARTER".to_string()
        },
        4,
        snap.is_running,
        starter_down,
        hovered == EnginePart::CrankHubStarter,
    );
}

// ============================================================================
// OPEN-SPACE HUD ELEMENTS (FRAMELESS OVERLAYS ON SINGLE CANVAS)
// ============================================================================

#[allow(clippy::too_many_arguments)]
fn draw_top_hud(
    ui: &mut egui::Ui,
    p: &egui::Painter,
    full_rect: Rect,
    tl: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    cam_yaw: &mut f32,
    cam_pitch: &mut f32,
    cam_zoom: &mut f32,
) {
    // Top-left title & audio controls floating directly on canvas
    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(tl), |ui| {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("F1 3.0L V10")
                    .monospace()
                    .strong()
                    .size(14.0)
                    .color(TEXT_PRIMARY),
            );
            ui.add_space(10.0);
            let mute_lbl = if ctrl.muted { "MUTED" } else { "AUDIO" };
            if ui.small_button(mute_lbl).clicked() {
                ctrl.muted = !ctrl.muted;
            }
            ui.add(egui::Slider::new(&mut ctrl.master_volume, 0.0..=1.5).show_value(false));
        });
        ui.horizontal(|ui| {
            if ui.small_button("Auto Start [Q]").clicked() {
                ctrl.quick_start_trigger = true;
            }
            if ui.small_button("Reset Engine").clicked() {
                ctrl.reset_repair_trigger = true;
            }
            if ui.small_button("Reset View [R]").clicked() {
                *cam_yaw = -0.62;
                *cam_pitch = 0.34;
                *cam_zoom = 1.0;
            }
        });
    });

    // Center Open-Space Tachometer LED Bar + Primary Numbers
    let center_x = full_rect.center().x;
    let bar_w = 270.0_f32;
    let bar_left = center_x - bar_w * 0.5;
    let led_y = full_rect.top() + 16.0;
    let flash_on = ((ui.input(|i| i.time) * 14.0) as i32) % 2 == 0;
    for i in 0..15 {
        let frac = i as f64 / 14.0;
        let thresh = 13_500.0 + frac * (ctrl.rev_limit_rpm - 13_500.0);
        let lit = snap.rpm >= thresh;
        let col = if !lit {
            Color32::from_rgb(22, 26, 33)
        } else if i < 5 {
            ACCENT_GREEN
        } else if i < 10 {
            ACCENT_AMBER
        } else if snap.rpm > ctrl.rev_limit_rpm - 450.0 && !flash_on {
            Color32::from_rgb(35, 42, 60)
        } else {
            ACCENT_CYAN
        };
        let seg = Rect::from_min_size(Pos2::new(bar_left + i as f32 * 18.2, led_y), Vec2::new(14.0, 6.0));
        p.rect_filled(seg, Rounding::same(1.0), col);
    }

    let gear_str = if snap.gear == 0 {
        "N".to_string()
    } else {
        format!("{}", snap.gear)
    };
    let cols: [(&str, String, Color32, f32); 5] = [
        (
            "RPM",
            format!("{:5.0}", snap.rpm),
            if snap.rev_limiter_active { ACCENT_RED } else { TEXT_PRIMARY },
            center_x - 190.0,
        ),
        ("POWER", format!("{:3.0} BHP", snap.power_bhp.max(0.0)), ACCENT_GREEN, center_x - 95.0),
        ("TORQUE", format!("{:3.0} Nm", snap.torque_nm.max(0.0)), ACCENT_AMBER, center_x + 5.0),
        ("GEAR", gear_str, ACCENT_CYAN, center_x + 95.0),
        ("SPEED", format!("{:3.0} km/h", snap.speed_kmh), TEXT_PRIMARY, center_x + 175.0),
    ];
    for (lbl, val, c, x) in &cols {
        p.text(Pos2::new(*x, led_y + 13.0), Align2::CENTER_TOP, *lbl, FontId::monospace(9.0), TEXT_MUTED);
        p.text(Pos2::new(*x, led_y + 25.0), Align2::CENTER_TOP, val, FontId::monospace(15.0), *c);
    }

    // Status Flag Top Right-of-Center
    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
    let (st_txt, st_col) = if snap.cold_seized {
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
    p.text(
        Pos2::new(full_rect.right() - 22.0, full_rect.top() + 20.0),
        Align2::RIGHT_TOP,
        st_txt,
        FontId::monospace(12.0),
        st_col,
    );
}

fn draw_vertical_throttle_slider(
    ui: &mut egui::Ui,
    p: &egui::Painter,
    bar_rect: Rect,
    hit_rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    dragging_bar: &mut bool,
) {
    // Label above slider
    p.text(
        Pos2::new(bar_rect.center().x, bar_rect.top() - 16.0),
        Align2::LEFT_BOTTOM,
        format!("THROTTLE {:3.0}%", ctrl.throttle * 100.0),
        FontId::monospace(11.0),
        TEXT_PRIMARY,
    );

    // Interactive region covering the vertical track
    let resp = ui.interact(hit_rect, ui.id().with("vert_throttle"), Sense::click_and_drag());
    if resp.drag_started() || resp.clicked() || (resp.is_pointer_button_down_on()) {
        *dragging_bar = true;
    }
    if !ui.ctx().input(|i| i.pointer.primary_down()) {
        *dragging_bar = false;
    }
    if *dragging_bar {
        if let Some(pos) = ui.ctx().input(|i| i.pointer.interact_pos()) {
            let norm = 1.0 - ((pos.y - bar_rect.top()) / bar_rect.height()).clamp(0.0, 1.0);
            ctrl.throttle = norm as f64;
        }
    }
    if resp.hovered() || *dragging_bar {
        ui.ctx().set_cursor_icon(CursorIcon::ResizeVertical);
    }

    // Track background & border
    p.rect_filled(bar_rect, 3.0, BG_PILL);
    p.rect_stroke(bar_rect, 3.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    // Requested throttle bar + thin effective throttle (IAC / blip) overlay
    let req_h = (ctrl.throttle as f32) * bar_rect.height();
    let eff_h = (snap.effective_throttle as f32) * bar_rect.height();
    if eff_h > 1.0 {
        let eff_rect = Rect::from_min_max(
            Pos2::new(bar_rect.left() + 2.0, bar_rect.bottom() - eff_h),
            Pos2::new(bar_rect.right() - 2.0, bar_rect.bottom()),
        );
        p.rect_filled(eff_rect, 2.0, Color32::from_rgba_unmultiplied(92, 180, 230, 55));
    }
    if req_h > 1.0 {
        let req_rect = Rect::from_min_max(
            Pos2::new(bar_rect.left() + 3.0, bar_rect.bottom() - req_h),
            Pos2::new(bar_rect.right() - 3.0, bar_rect.bottom() - 2.0),
        );
        p.rect_filled(req_rect, 2.0, ACCENT_GREEN);
    }

    // Grab handle line
    let handle_y = bar_rect.bottom() - req_h;
    p.line_segment(
        [Pos2::new(bar_rect.left() - 3.0, handle_y), Pos2::new(bar_rect.right() + 3.0, handle_y)],
        Stroke::new(3.0_f32, TEXT_PRIMARY),
    );

    // Tick marks & Keybind markings along the right side of the vertical slider
    let marks = [
        (1.00_f32, "100%  [SPACE] WOT", ACCENT_GREEN),
        (0.75_f32, " 75%  [W / ↑] +", TEXT_MUTED),
        (0.50_f32, " 50%  Half Load", TEXT_MUTED),
        (0.25_f32, " 25%  [S / ↓] -", TEXT_MUTED),
        (0.00_f32, "  0%  Idle Cut", TEXT_MUTED),
    ];
    for &(frac, label, col) in &marks {
        let ty = bar_rect.bottom() - frac * bar_rect.height();
        p.line_segment(
            [Pos2::new(bar_rect.right() + 2.0, ty), Pos2::new(bar_rect.right() + 10.0, ty)],
            Stroke::new(1.0_f32, BORDER_SUBTLE),
        );
        p.text(
            Pos2::new(bar_rect.right() + 14.0, ty),
            Align2::LEFT_CENTER,
            label,
            FontId::monospace(10.0),
            col,
        );
    }

    // Additional keybind legend lines neatly stacked near slider
    let kb_top = bar_rect.bottom() - bar_rect.height() * 0.62;
    let kb_x = bar_rect.right() + 14.0;
    let extra_binds = [
        "[Z / X]    Shift Down / Up",
        "[MMB Drag] Orbit 3D Engine",
        "[Wheel]    Zoom In / Out",
        "[Q / R]    Auto Start / View",
    ];
    for (idx, line) in extra_binds.iter().enumerate() {
        p.text(
            Pos2::new(kb_x, kb_top + idx as f32 * 14.0),
            Align2::LEFT_TOP,
            *line,
            FontId::monospace(9.5),
            Color32::from_rgb(95, 104, 118),
        );
    }
}

fn draw_bottom_left_hud(
    ui: &mut egui::Ui,
    rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    show_ecu: &mut bool,
) -> bool {
    let mut starter_btn_down = false;
    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(rect), |ui| {
        let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
        let warm_ok = min_temp >= 60.0;
        let pneu_ok = ctrl.n2_pressure_bar >= 100.0;
        let armed_ok = ctrl.armed();

        let (guide, gcol) = if snap.cold_seized {
            ("SEIZED COLD (<60C) — Pre-heat block first!", ACCENT_RED)
        } else if !warm_ok && !ctrl.preheat_on() {
            ("1. Click 3D Block (or pill) to Pre-Heat 60C", ACCENT_AMBER)
        } else if !warm_ok {
            ("Warming block fluids to 60C...", ACCENT_AMBER)
        } else if !pneu_ok {
            ("2. Click 3D N2 Bottle to pressurize valves", ACCENT_AMBER)
        } else if !armed_ok {
            ("3. Click 3D V-Valley Rail to arm Fuel & Spark", ACCENT_AMBER)
        } else if !snap.is_running {
            ("4. Hold 3D Rear Starter (or HOLD CRANK)", ACCENT_GREEN)
        } else {
            ("ENGINE RUNNING — Use Throttle or Auto Mode", ACCENT_GREEN)
        };
        ui.label(RichText::new(guide).monospace().size(10.5).color(gcol));
        ui.add_space(4.0);

        // Quick backup/accessibility startup pills (synchronized with 3D model parts)
        ui.horizontal(|ui| {
            let pre_col = if warm_ok { ACCENT_GREEN } else if ctrl.preheat_on() { ACCENT_AMBER } else { TEXT_PRIMARY };
            if ui.add_sized([58.0, 21.0], egui::Button::new(RichText::new(format!("1 {:.0}C", min_temp)).monospace().size(9.5).color(pre_col))).clicked() {
                ctrl.set_preheat(!ctrl.preheat_on());
            }
            if ui.add_sized([56.0, 21.0], egui::Button::new(RichText::new("2 N2").monospace().size(9.5).color(if pneu_ok { ACCENT_GREEN } else { TEXT_PRIMARY }))).clicked() {
                ctrl.toggle_pneumatics();
            }
            if ui.add_sized([58.0, 21.0], egui::Button::new(RichText::new("3 ARM").monospace().size(9.5).color(if armed_ok { ACCENT_GREEN } else { TEXT_PRIMARY }))).clicked() {
                ctrl.toggle_arm();
            }
            let sb = ui.add_sized(
                [66.0, 21.0],
                egui::Button::new(RichText::new("4 CRANK").monospace().size(9.5).color(if ctrl.starter_active { ACCENT_AMBER } else { TEXT_PRIMARY })),
            );
            starter_btn_down = sb.is_pointer_button_down_on();
        });

        ui.add_space(6.0);

        // Open-space Test Mode Pills
        egui::Grid::new("open_modes")
            .num_columns(2)
            .spacing([5.0, 4.0])
            .show(ui, |ui| {
                let modes = [
                    (BenchMode::FreeRev, "Manual Rev"),
                    (BenchMode::DynoSweep, "Auto Dyno Pull"),
                    (BenchMode::MonzaHotlapAuto, "Auto Monza Lap"),
                    (BenchMode::TrackManualGearbox, "7-Spd Gearbox"),
                ];
                for (i, (m, lbl)) in modes.iter().enumerate() {
                    let sel = ctrl.bench_mode == *m;
                    if ui.add_sized([120.0, 21.0], egui::SelectableLabel::new(sel, *lbl)).clicked() {
                        ctrl.bench_mode = *m;
                        if *m == BenchMode::DynoSweep {
                            ctrl.start_dyno_sweep = true;
                        } else if *m == BenchMode::MonzaHotlapAuto && !snap.is_running {
                            ctrl.quick_start_trigger = true;
                        }
                    }
                    if i % 2 == 1 {
                        ui.end_row();
                    }
                }
            });

        ui.add_space(4.0);
        match ctrl.bench_mode {
            BenchMode::TrackManualGearbox => {
                ui.horizontal(|ui| {
                    if ui.small_button("- [Z]").clicked() {
                        ctrl.requested_gear = ctrl.requested_gear.saturating_sub(1).max(1);
                    }
                    ui.label(RichText::new(format!("GEAR {}", ctrl.requested_gear)).monospace().strong());
                    if ui.small_button("+ [X]").clicked() {
                        ctrl.requested_gear = (ctrl.requested_gear + 1).min(7);
                    }
                });
            }
            BenchMode::MonzaHotlapAuto => {
                ui.label(
                    RichText::new(format!("{} ({:.0} m)", snap.monza_sector_name, snap.monza_dist_m))
                        .monospace()
                        .size(10.0)
                        .color(ACCENT_CYAN),
                );
            }
            BenchMode::DynoSweep => {
                if ui.small_button("Re-Run Auto Dyno Pull").clicked() {
                    ctrl.start_dyno_sweep = true;
                }
                if snap.dyno_sweep_running {
                    ui.label(
                        RichText::new("Sweeping 5.5k -> 19.5k RPM...")
                            .monospace()
                            .size(10.0)
                            .color(ACCENT_GREEN),
                    );
                }
            }
            _ => {}
        }

        ui.add_space(4.0);
        if ui.small_button(if *show_ecu { "Hide ECU Tuning" } else { "ECU & Dyno Tuning..." }).clicked() {
            *show_ecu = !*show_ecu;
        }
        if *show_ecu {
            ui.horizontal(|ui| {
                if ui.selectable_label(ctrl.bench_mode == BenchMode::DynoManualBrake, "Brake").clicked() {
                    ctrl.bench_mode = BenchMode::DynoManualBrake;
                }
                if ui.selectable_label(ctrl.bench_mode == BenchMode::DynoRpmHold, "RPM Hold").clicked() {
                    ctrl.bench_mode = BenchMode::DynoRpmHold;
                }
            });
            if ctrl.bench_mode == BenchMode::DynoManualBrake {
                ui.add(egui::Slider::new(&mut ctrl.dyno_brake_load, 0.0..=1.0).text("Brake"));
            } else if ctrl.bench_mode == BenchMode::DynoRpmHold {
                ui.add(egui::Slider::new(&mut ctrl.dyno_hold_rpm, 4000.0..=19500.0).step_by(250.0).text("Hold"));
            }
            ui.add(egui::Slider::new(&mut ctrl.lambda_target, 0.75..=1.20).text("Lambda"));
            ui.add(egui::Slider::new(&mut ctrl.spark_offset_deg, -15.0..=15.0).text("Spark°"));
            ui.checkbox(&mut ctrl.trumpets_auto, "Auto Trumpets");
            if !ctrl.trumpets_auto {
                ui.add(egui::Slider::new(&mut ctrl.manual_trumpet_mm, 75.0..=135.0).text("mm"));
            }
        }
    });
    starter_btn_down
}

fn draw_right_hud(
    ui: &mut egui::Ui,
    rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    diag_plot: &mut DiagnosticPlot,
) {
    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(rect), |ui| {
        let (peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm) = compute_dyno_peaks(snap);
        ui.horizontal(|ui| {
            ui.label(RichText::new("DYNO").monospace().strong().size(10.5).color(TEXT_MUTED));
            ui.label(
                RichText::new(format!("{:.0}BHP@{:.0} | {:.0}Nm@{:.0}", peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm))
                    .monospace()
                    .size(9.5)
                    .color(TEXT_PRIMARY),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.small_button("Clr").clicked() {
                    ctrl.clear_dyno_trace = true;
                }
            });
        });
        draw_frameless_dyno_plot(ui, snap);

        ui.add_space(8.0);

        ui.horizontal(|ui| {
            if ui.selectable_label(*diag_plot == DiagnosticPlot::PvDiagram, "P-V LOOP").clicked() {
                *diag_plot = DiagnosticPlot::PvDiagram;
            }
            if ui.selectable_label(*diag_plot == DiagnosticPlot::Waveform, "EXHAUST WAVE").clicked() {
                *diag_plot = DiagnosticPlot::Waveform;
            }
        });
        match *diag_plot {
            DiagnosticPlot::PvDiagram => draw_frameless_pv_plot(ui, snap),
            DiagnosticPlot::Waveform => draw_frameless_scope_plot(ui, snap),
        }

        ui.add_space(8.0);
        ui.label(RichText::new("TELEMETRY").monospace().strong().size(10.5).color(TEXT_MUTED));
        egui::Grid::new("open_vitals")
            .num_columns(2)
            .spacing([14.0, 3.0])
            .show(ui, |ui| {
                vital_row(ui, "Block / Oil", &format!("{:.0}°C / {:.0}°C", snap.coolant_temp_c, snap.oil_temp_c), snap.coolant_temp_c > 112.0);
                vital_row(ui, "Exhaust EGT", &format!("{:.0} °C", snap.egt_c), snap.egt_c > 980.0);
                vital_row(ui, "Valve N2", &format!("{:.0} / {:.0} bar", ctrl.n2_pressure_bar, snap.required_n2_bar), snap.valve_float_active);
                vital_row(ui, "Plenum / VE", &format!("{:.2}b / {:.0}%", snap.plenum_pressure_bar, snap.ve_percent), false);
                vital_row(ui, "Trumpets", &format!("{:.1} mm", snap.trumpet_length_mm), false);
                if snap.bore_health < 0.99 || snap.knock_count > 0 {
                    vital_row(ui, "Bore / Knocks", &format!("{:.0}% / {}", snap.bore_health * 100.0, snap.knock_count), true);
                }
            });
    });
}

fn vital_row(ui: &mut egui::Ui, label: &str, value: &str, warn: bool) {
    ui.label(RichText::new(label).monospace().size(10.0).color(TEXT_MUTED));
    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
        ui.label(RichText::new(value).monospace().size(10.5).color(if warn { ACCENT_RED } else { TEXT_PRIMARY }));
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

fn draw_frameless_dyno_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(284.0, 150.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.line_segment(
        [Pos2::new(rect.left(), rect.bottom() - 1.0), Pos2::new(rect.right(), rect.bottom() - 1.0)],
        Stroke::new(1.0_f32, BORDER_SUBTLE),
    );
    p.line_segment(
        [Pos2::new(rect.left() + 1.0, rect.top()), Pos2::new(rect.left() + 1.0, rect.bottom())],
        Stroke::new(1.0_f32, BORDER_SUBTLE),
    );

    let pad = rect.shrink(8.0);
    for i in 1..4 {
        let y = pad.bottom() - (i as f32 / 4.0) * pad.height();
        p.line_segment([Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)], Stroke::new(0.6_f32, Color32::from_rgb(18, 22, 28)));
    }

    let rpm_span = (DYNO_MAX_RPM - DYNO_MIN_RPM) as f32;
    let mut bhp_pts = Vec::new();
    let mut nm_pts = Vec::new();
    for i in 0..DYNO_BINS {
        let pt = snap.dyno_curve[i];
        if pt.recorded {
            let mut sum_p = pt.power_bhp;
            let mut sum_t = pt.torque_nm;
            let mut w = 1.0f32;
            if i > 0 && snap.dyno_curve[i - 1].recorded {
                sum_p += 0.45 * snap.dyno_curve[i - 1].power_bhp;
                sum_t += 0.45 * snap.dyno_curve[i - 1].torque_nm;
                w += 0.45;
            }
            if i + 1 < DYNO_BINS && snap.dyno_curve[i + 1].recorded {
                sum_p += 0.45 * snap.dyno_curve[i + 1].power_bhp;
                sum_t += 0.45 * snap.dyno_curve[i + 1].torque_nm;
                w += 0.45;
            }
            let nx = ((pt.rpm - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
            let x = pad.left() + nx * pad.width();
            bhp_pts.push(Pos2::new(x, pad.bottom() - ((sum_p / w) / 1000.0).clamp(0.0, 1.0) * pad.height()));
            nm_pts.push(Pos2::new(x, pad.bottom() - ((sum_t / w) / 450.0).clamp(0.0, 1.0) * pad.height()));
        }
    }
    if nm_pts.len() >= 2 {
        p.add(egui::Shape::line(nm_pts, Stroke::new(1.4_f32, ACCENT_AMBER)));
    }
    if bhp_pts.len() >= 2 {
        p.add(egui::Shape::line(bhp_pts, Stroke::new(1.8_f32, ACCENT_GREEN)));
    }
    if snap.rpm >= DYNO_MIN_RPM as f64 {
        let nx = (((snap.rpm as f32) - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
        let x = pad.left() + nx * pad.width();
        let y_bhp = pad.bottom() - ((snap.power_bhp as f32) / 1000.0).clamp(0.0, 1.0) * pad.height();
        p.circle_filled(Pos2::new(x, y_bhp), 3.0, TEXT_PRIMARY);
    }
    p.text(Pos2::new(pad.left() + 4.0, pad.top()), Align2::LEFT_TOP, "1000 BHP / 450 Nm", FontId::monospace(9.0), TEXT_MUTED);
    p.text(Pos2::new(pad.right(), pad.bottom() - 2.0), Align2::RIGHT_BOTTOM, "5k-20k RPM", FontId::monospace(9.0), TEXT_MUTED);
}

fn draw_frameless_pv_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(284.0, 140.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.line_segment(
        [Pos2::new(rect.left(), rect.bottom() - 1.0), Pos2::new(rect.right(), rect.bottom() - 1.0)],
        Stroke::new(1.0_f32, BORDER_SUBTLE),
    );
    p.line_segment(
        [Pos2::new(rect.left() + 1.0, rect.top()), Pos2::new(rect.left() + 1.0, rect.bottom())],
        Stroke::new(1.0_f32, BORDER_SUBTLE),
    );

    let pad = rect.shrink(8.0);
    let v_min = (CLEARANCE_VOL_M3 * 1e6) as f32 * 0.9;
    let v_max = ((CLEARANCE_VOL_M3 + DISP_VOL_M3) * 1e6) as f32 * 1.03;
    let map_pt = |v: f32, pr: f32| -> Pos2 {
        let nx = ((v - v_min) / (v_max - v_min)).clamp(0.0, 1.0);
        let ny = (pr / 105.0).clamp(0.0, 1.0);
        Pos2::new(pad.left() + nx * pad.width(), pad.bottom() - ny * pad.height())
    };
    let mut pts = Vec::with_capacity(PV_POINTS + 1);
    for &(v, pr) in &snap.pv_trace {
        pts.push(map_pt(v, pr));
    }
    if let Some(&first) = pts.first() {
        pts.push(first);
    }
    p.add(egui::Shape::line(pts, Stroke::new(1.5_f32, ACCENT_CYAN)));
    let cur_v = ((CLEARANCE_VOL_M3 + snap.cylinders[0].piston_norm * DISP_VOL_M3) * 1e6) as f32;
    p.circle_filled(map_pt(cur_v, snap.cylinders[0].pressure_bar as f32), 3.0, ACCENT_AMBER);
    p.text(
        Pos2::new(pad.left() + 4.0, pad.top()),
        Align2::LEFT_TOP,
        format!("{:.1} bar | {:.0} K", snap.cylinders[0].pressure_bar, snap.cylinders[0].temp_k),
        FontId::monospace(9.0),
        TEXT_MUTED,
    );
}

fn draw_frameless_scope_plot(ui: &mut egui::Ui, snap: &TelemetrySnapshot) {
    let (rect, _) = ui.allocate_exact_size(Vec2::new(284.0, 140.0), Sense::hover());
    let p = ui.painter_at(rect);
    let pad = rect.shrink(8.0);
    let cy = pad.center().y;
    p.line_segment([Pos2::new(pad.left(), cy), Pos2::new(pad.right(), cy)], Stroke::new(0.8_f32, Color32::from_rgb(22, 26, 33)));

    let mut pts_l = Vec::with_capacity(SCOPE_LEN);
    let mut pts_r = Vec::with_capacity(SCOPE_LEN);
    let mut pts_a = Vec::with_capacity(SCOPE_LEN);
    let amp = pad.height() * 0.44;
    for i in 0..SCOPE_LEN {
        let x = pad.left() + (i as f32 / (SCOPE_LEN - 1) as f32) * pad.width();
        pts_l.push(Pos2::new(x, cy - snap.scope_left[i].clamp(-1.0, 1.0) * amp));
        pts_r.push(Pos2::new(x, cy - snap.scope_right[i].clamp(-1.0, 1.0) * amp));
        pts_a.push(Pos2::new(x, cy - snap.scope_audio[i].clamp(-1.0, 1.0) * amp));
    }
    p.add(egui::Shape::line(pts_l, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(92, 180, 230, 85))));
    p.add(egui::Shape::line(pts_r, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(222, 158, 62, 85))));
    p.add(egui::Shape::line(pts_a, Stroke::new(1.4_f32, TEXT_PRIMARY)));
}

fn gas_color(cyl: &crate::sim::CylinderSnapshot) -> Color32 {
    if cyl.spark_firing || cyl.combustion_intensity > 0.85 {
        return Color32::from_rgba_unmultiplied(255, 224, 120, 210);
    }
    match cyl.phase {
        StrokePhase::Intake => Color32::from_rgba_unmultiplied(72, 170, 235, (65.0 + cyl.pressure_bar as f32 * 45.0).clamp(55.0, 150.0) as u8),
        StrokePhase::Compression => {
            let pf = (cyl.pressure_bar as f32 / 35.0).clamp(0.0, 1.0);
            Color32::from_rgba_unmultiplied((80.0 + pf * 165.0) as u8, (150.0 - pf * 20.0) as u8, (215.0 - pf * 165.0) as u8, 170)
        }
        StrokePhase::Power => {
            let i = cyl.combustion_intensity as f32;
            Color32::from_rgba_unmultiplied(250, (95.0 + i * 140.0) as u8, (30.0 + i * 105.0) as u8, (130.0 + i * 95.0) as u8)
        }
        StrokePhase::Exhaust => Color32::from_rgba_unmultiplied(210, 65, 48, 110),
    }
}

fn header_glow_color(egt_c: f64) -> Color32 {
    let t = ((egt_c - 320.0) / 650.0).clamp(0.0, 1.0) as f32;
    Color32::from_rgb(
        (58.0 + t * 188.0) as u8,
        (62.0 + t * 88.0) as u8,
        (70.0 - t * 44.0) as u8,
    )
}
