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

// Pure Obsidian Black Palette
const BG_CANVAS: Color32 = Color32::from_rgb(5, 6, 8);
const BG_PILL: Color32 = Color32::from_rgb(14, 17, 23);
const BORDER_SUBTLE: Color32 = Color32::from_rgb(28, 33, 42);
const TEXT_PRIMARY: Color32 = Color32::from_rgb(228, 232, 240);
const TEXT_MUTED: Color32 = Color32::from_rgb(118, 126, 142);
const ACCENT_GREEN: Color32 = Color32::from_rgb(72, 188, 120);
const ACCENT_AMBER: Color32 = Color32::from_rgb(222, 158, 62);
const ACCENT_RED: Color32 = Color32::from_rgb(212, 74, 74);
const ACCENT_CYAN: Color32 = Color32::from_rgb(92, 180, 230);

#[inline]
fn bg_card() -> Color32 {
    Color32::from_rgba_unmultiplied(10, 12, 17, 205)
}

#[inline]
fn bg_popover() -> Color32 {
    Color32::from_rgba_unmultiplied(12, 15, 21, 242)
}

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
    menu_open: bool,
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
            menu_open: false,
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

        // Replace default programmer font with DIN 1451 (Bahnschrift) / Segoe UI motorsport typeface
        if let Ok(bytes) = std::fs::read(r"C:\Windows\Fonts\bahnschrift.ttf")
            .or_else(|_| std::fs::read(r"C:\Windows\Fonts\segoeui.ttf"))
        {
            let mut fonts = egui::FontDefinitions::default();
            fonts
                .font_data
                .insert("din".to_owned(), egui::FontData::from_owned(bytes));
            if let Some(fam) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
                fam.insert(0, "din".to_owned());
            }
            if let Some(fam) = fonts.families.get_mut(&egui::FontFamily::Monospace) {
                fam.insert(0, "din".to_owned());
            }
            ctx.set_fonts(fonts);
        }

        let mut style = (*ctx.style()).clone();
        style.visuals.window_fill = BG_CANVAS;
        style.visuals.panel_fill = BG_CANVAS;
        style.visuals.extreme_bg_color = BG_CANVAS;
        style.visuals.faint_bg_color = BG_PILL;
        style.visuals.widgets.noninteractive.bg_fill = Color32::TRANSPARENT;
        style.visuals.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.inactive.bg_fill = BG_PILL;
        style.visuals.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, BORDER_SUBTLE);
        style.visuals.widgets.hovered.bg_fill = Color32::from_rgb(24, 29, 38);
        style.visuals.widgets.hovered.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(56, 65, 82));
        style.visuals.widgets.active.bg_fill = Color32::from_rgb(34, 41, 54);
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
            if i.key_pressed(egui::Key::Escape) {
                ctrl.reset_repair_trigger = true;
                self.cam_yaw = -0.62;
                self.cam_pitch = 0.34;
                self.cam_zoom = 1.0;
                self.menu_open = false;
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

        egui::CentralPanel::default()
            .frame(egui::Frame::none().fill(BG_CANVAS))
            .show(ctx, |ui| {
                let full_rect = ui.max_rect();
                let painter = ui.painter_at(full_rect);

                let arena_center = Pos2::new(
                    full_rect.left() + full_rect.width() * 0.45,
                    full_rect.top() + full_rect.height() * 0.55,
                );
                let arena_scale_base = (full_rect.height() * 0.88).min(full_rect.width() * 0.58);

                // 1. CAMERA ORBIT (Middle Mouse Button Drag + Secondary Fallback + Wheel Zoom)
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

                // 2. HUD LAYOUT RECTS
                let throttle_h = (full_rect.height() * 0.44).clamp(240.0, 380.0);
                let throttle_top = full_rect.center().y - throttle_h * 0.52;
                let throttle_bar_rect =
                    Rect::from_min_size(Pos2::new(full_rect.left() + 28.0, throttle_top), Vec2::new(24.0, throttle_h));
                let throttle_hit_rect = throttle_bar_rect.expand2(Vec2::new(14.0, 10.0));

                let hamburger_btn_rect = Rect::from_min_size(
                    Pos2::new(full_rect.left() + 26.0, full_rect.bottom() - 48.0),
                    Vec2::new(186.0, 32.0),
                );
                let popover_rect = Rect::from_min_size(
                    Pos2::new(full_rect.left() + 26.0, hamburger_btn_rect.top() - 252.0),
                    Vec2::new(242.0, 244.0),
                );

                let right_w = 332.0_f32;
                let right_hud_rect = Rect::from_min_max(
                    Pos2::new(full_rect.right() - right_w - 18.0, full_rect.top() + 74.0),
                    Pos2::new(full_rect.right() - 18.0, full_rect.bottom() - 20.0),
                );

                let pointer_pos = ctx.input(|i| i.pointer.interact_pos());
                let hover_pos = ctx.input(|i| i.pointer.hover_pos());
                let menu_was_open = self.menu_open;
                let pos_over_hud = |p: Pos2| {
                    throttle_hit_rect.contains(p)
                        || hamburger_btn_rect.contains(p)
                        || (menu_was_open && popover_rect.contains(p))
                        || right_hud_rect.contains(p)
                };

                // 3. BUILD & RENDER LOW-POLY 3D V10 (first pass projects hit targets for hover highlights)
                let cam = Camera3D::new(
                    arena_center,
                    arena_scale_base * self.cam_zoom,
                    self.cam_yaw,
                    self.cam_pitch,
                );

                // Compute hover hit from last known target geometry during interaction after build:
                // We build with tentative active_highlight, then interact cleanly below.
                let mut scene = Scene3D::default();
                let pre_highlight = if self.active_drag_part != EnginePart::None {
                    self.active_drag_part
                } else {
                    EnginePart::None
                };
                let hits = build_v10_scene_3d(&cam, &mut scene, &snap, &ctrl, pre_highlight);

                let hovered_part = hover_pos
                    .filter(|p| !pos_over_hud(*p))
                    .map(|p| hits.hit_test(p))
                    .unwrap_or(EnginePart::None);
                let active_highlight = if self.active_drag_part != EnginePart::None {
                    self.active_drag_part
                } else {
                    hovered_part
                };

                // Re-shade cleanly if hovering an interactive 3D part so the part glows cleanly
                if hovered_part != EnginePart::None && hovered_part != pre_highlight {
                    scene.items.clear();
                    let _ = build_v10_scene_3d(&cam, &mut scene, &snap, &ctrl, active_highlight);
                }
                scene.render(&painter);

                // 4. DIRECT 3D ENGINE INTERACTION LAYER
                let engine_resp = ui.interact(
                    full_rect,
                    ui.id().with("engine_3d_space"),
                    Sense::click_and_drag(),
                );

                if engine_resp.drag_started_by(egui::PointerButton::Primary) {
                    if let Some(pos) = pointer_pos.filter(|p| !pos_over_hud(*p)) {
                        self.menu_open = false;
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
                        self.menu_open = false;
                        match hits.hit_test(pos) {
                            EnginePart::CylinderBlock => ctrl.set_preheat(!ctrl.preheat_on()),
                            EnginePart::PneumaticHeads => ctrl.toggle_pneumatics(),
                            EnginePart::SparkPlugs => ctrl.toggle_arm(),
                            _ => {}
                        }
                    }
                }

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
                ctrl.starter_active = starter_down;

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

                // 5. TOP OPEN-SPACE TACHOMETER & SHIFT LIGHTS (clean, buttonless top bar)
                draw_top_hud(ui, &painter, full_rect, &snap, &ctrl);

                // 6. LEFT VERTICAL THROTTLE SLIDER + KEYBIND MARKINGS
                draw_vertical_throttle_slider(
                    ui,
                    &painter,
                    throttle_bar_rect,
                    throttle_hit_rect,
                    &snap,
                    &mut ctrl,
                    &mut self.dragging_throttle_bar,
                );

                // 7. RIGHT-SIDE CLEAN TELEMETRY GRAPHS
                draw_right_hud(ui, right_hud_rect, &snap, &mut ctrl, &mut self.diag_plot);

                // 8. BOTTOM-LEFT HAMBURGER MENU (Manual Rev, Auto Pull, Monza Lap, 7-Spd, Audio, Utilities)
                draw_bottom_hamburger_menu(
                    ui,
                    &painter,
                    hamburger_btn_rect,
                    popover_rect,
                    &snap,
                    &mut ctrl,
                    &mut self.menu_open,
                    &mut self.cam_yaw,
                    &mut self.cam_pitch,
                    &mut self.cam_zoom,
                );

                // 9. SINGLE CLEAN BOTTOM-CENTER INTERACTIVE GUIDE / PART PROMPT
                draw_footer_prompt(&painter, arena_center.x, full_rect.bottom() - 14.0, &snap, &ctrl, active_highlight);
            });

        *self.controls.lock().unwrap() = ctrl;
    }
}

// ============================================================================
// 3D CAMERA & ARTIFACT-FREE LOW-POLY RENDERER
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

        let fwd = [-sy * cp, -sp, cy * cp];
        let right = [cy, 0.0, sy];
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
            target: [0.0, 0.13, 0.0],
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
        if z_cam < 0.35 {
            return None;
        }
        let persp = self.scale_px / z_cam;
        let sx = self.origin_2d.x + x_cam * persp;
        let sy = self.origin_2d.y - y_cam * persp;
        if !sx.is_finite() || !sy.is_finite() || sx.abs() > 8000.0 || sy.abs() > 8000.0 {
            return None;
        }
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
fn tint_mix(a: Color32, b: Color32, t: f32) -> Color32 {
    let u = t.clamp(0.0, 1.0);
    Color32::from_rgba_unmultiplied(
        ((a.r() as f32) * (1.0 - u) + (b.r() as f32) * u) as u8,
        ((a.g() as f32) * (1.0 - u) + (b.g() as f32) * u) as u8,
        ((a.b() as f32) * (1.0 - u) + (b.b() as f32) * u) as u8,
        a.a(),
    )
}

#[inline]
fn shade_color(base: Color32, normal: [f32; 3]) -> Color32 {
    let key = norm3([-0.38, 0.86, -0.34]);
    let rim = norm3([0.55, -0.30, 0.78]);
    let n = norm3(normal);
    let k = dot3(n, key).abs();
    let r = dot3(n, rim).abs();
    let gain = 0.50 + 0.44 * k + 0.14 * r;
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
    /// Push a strokeless, winding-normalized 3D convex polygon (eliminates epaint miter/feather spikes)
    fn push_poly(&mut self, cam: &Camera3D, world_pts: &[[f32; 3]], fill: Color32, shade: bool) {
        let n_verts = world_pts.len();
        if n_verts < 3 {
            return;
        }
        let mut pts = Vec::with_capacity(n_verts);
        let mut depth_sum = 0.0_f32;
        for &wp in world_pts {
            if let Some((p2, z, _)) = cam.project(wp) {
                pts.push(p2);
                depth_sum += z;
            } else {
                return;
            }
        }

        // Signed 2D area & edge-length guard: cull near-collinear slivers and enforce CCW winding
        let mut area2 = 0.0_f32;
        for i in 0..n_verts {
            let a = pts[i];
            let b = pts[(i + 1) % n_verts];
            area2 += a.x * b.y - b.x * a.y;
        }
        if area2.abs() < 2.2 {
            return;
        }
        if area2 < 0.0 {
            pts.reverse();
        }

        let final_fill = if shade {
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
            depth: depth_sum / n_verts as f32,
            prim: Prim3D::Poly { pts, fill: final_fill },
        });
    }

    fn push_box_3d(
        &mut self,
        cam: &Camera3D,
        center: [f32; 3],
        ax_x: [f32; 3],
        ax_y: [f32; 3],
        ax_z: [f32; 3],
        hx: f32,
        hy: f32,
        hz: f32,
        fill: Color32,
    ) {
        let vx = mul3(ax_x, hx);
        let vy = mul3(ax_y, hy);
        let vz = mul3(ax_z, hz);

        let p000 = sub3(sub3(sub3(center, vx), vy), vz);
        let p100 = sub3(sub3(add3(center, vx), vy), vz);
        let p110 = sub3(add3(add3(center, vx), vy), vz);
        let p010 = sub3(add3(sub3(center, vx), vy), vz);
        let p001 = add3(sub3(sub3(center, vx), vy), vz);
        let p101 = add3(sub3(add3(center, vx), vy), vz);
        let p111 = add3(add3(add3(center, vx), vy), vz);
        let p011 = add3(add3(sub3(center, vx), vy), vz);

        let faces = [
            [p000, p100, p110, p010], // -z
            [p001, p011, p111, p101], // +z
            [p000, p010, p011, p001], // -x
            [p100, p101, p111, p110], // +x
            [p000, p001, p101, p100], // -y
            [p010, p110, p111, p011], // +y
        ];
        for f in &faces {
            self.push_poly(cam, f, fill, true);
        }
    }

    /// 3D shaded segment rendered safely (falls back to disc cap when viewed head-on to avoid line normal glitches)
    fn push_rod(&mut self, cam: &Camera3D, a_w: [f32; 3], b_w: [f32; 3], radius_m: f32, col: Color32) {
        if let (Some((pa, za, s_a)), Some((pb, zb, s_b))) = (cam.project(a_w), cam.project(b_w)) {
            let r_px = (radius_m * 0.5 * (s_a + s_b)).clamp(0.7, 14.0);
            let len_sq = (pb - pa).length_sq();
            if len_sq < 1.5 {
                self.items.push(DrawItem3D {
                    depth: 0.5 * (za + zb),
                    prim: Prim3D::Circle { c: pa, r: r_px, fill: col },
                });
            } else {
                self.items.push(DrawItem3D {
                    depth: 0.5 * (za + zb),
                    prim: Prim3D::Line {
                        a: pa,
                        b: pb,
                        stroke: Stroke::new(r_px * 2.0, col),
                    },
                });
            }
        }
    }

    fn push_line_3d(&mut self, cam: &Camera3D, a_w: [f32; 3], b_w: [f32; 3], stroke: Stroke) {
        if let (Some((pa, za, _)), Some((pb, zb, _))) = (cam.project(a_w), cam.project(b_w)) {
            if (pb - pa).length_sq() >= 1.2 {
                self.items.push(DrawItem3D {
                    depth: 0.5 * (za + zb),
                    prim: Prim3D::Line { a: pa, b: pb, stroke },
                });
            }
        }
    }

    fn push_disc_3d(
        &mut self,
        cam: &Camera3D,
        center: [f32; 3],
        u_dir: [f32; 3],
        v_dir: [f32; 3],
        radius: f32,
        segs: usize,
        fill: Color32,
        shade: bool,
    ) {
        let mut pts = Vec::with_capacity(segs);
        for k in 0..segs {
            let a = (k as f32 / segs as f32) * 2.0 * PI;
            pts.push(add3(
                center,
                add3(mul3(u_dir, a.cos() * radius), mul3(v_dir, a.sin() * radius)),
            ));
        }
        self.push_poly(cam, &pts, fill, shade);
    }

    fn push_cylinder_3d(
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
        cap_ends: bool,
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
            self.push_poly(cam, &q, fill, shade);
        }
        if cap_ends {
            self.push_disc_3d(cam, p0, u_dir, v_dir, r0, segs, fill, shade);
            self.push_disc_3d(cam, p1, u_dir, v_dir, r1, segs, fill, shade);
        }
    }

    fn push_spark(&mut self, cam: &Camera3D, c_w: [f32; 3], radius_m: f32, fill: Color32) {
        if let Some((pc, zc, s)) = cam.project(c_w) {
            self.items.push(DrawItem3D {
                depth: zc,
                prim: Prim3D::Circle {
                    c: pc,
                    r: (radius_m * s).clamp(2.0, 26.0),
                    fill,
                },
            });
        }
    }

    fn render(&mut self, p: &egui::Painter) {
        self.items
            .sort_by(|a, b| b.depth.partial_cmp(&a.depth).unwrap_or(std::cmp::Ordering::Equal));
        for item in &self.items {
            match &item.prim {
                Prim3D::Poly { pts, fill } => {
                    p.add(egui::Shape::convex_polygon(pts.clone(), *fill, Stroke::NONE));
                }
                Prim3D::Line { a, b, stroke } => {
                    p.line_segment([*a, *b], *stroke);
                }
                Prim3D::Circle { c, r, fill } => {
                    p.circle_filled(*c, *r, *fill);
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
}

impl HitTargets3D {
    fn hit_test(&self, pos: Pos2) -> EnginePart {
        if (pos - self.starter_pos).length() < 36.0 {
            return EnginePart::CrankHubStarter;
        }
        if (pos - self.front_pulley_pos).length() < 38.0 {
            return EnginePart::FlywheelHandCrank;
        }
        for &p in &self.n2_pts {
            if (pos - p).length() < 26.0 {
                return EnginePart::PneumaticHeads;
            }
        }
        for &p in &self.rail_pts {
            if (pos - p).length() < 24.0 {
                return EnginePart::SparkPlugs;
            }
        }
        for &p in &self.trumpet_pts {
            if (pos - p).length() < 26.0 {
                return EnginePart::IntakeTrumpets;
            }
        }
        for &p in &self.preheat_pts {
            if (pos - p).length() < 30.0 {
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
    highlight: EnginePart,
) -> HitTargets3D {
    let bore = BORE_M as f32;
    let r_crank = CRANK_RADIUS_M as f32;
    let l_rod = CONROD_M as f32;
    let stroke = STROKE_M as f32;
    let half_v = (V_ANGLE_DEG as f32 * 0.5).to_radians();
    let throw_spacing = 0.112_f32;
    let z_front = -2.0 * throw_spacing - 0.055;
    let z_rear = 2.0 * throw_spacing + 0.055;
    let bay_hz = throw_spacing * 0.48;

    let tdc_wrist = r_crank + l_rod;
    let piston_h = bore * 0.28;
    let crown_offset = piston_h * 0.46;
    let head_dist = tdc_wrist + crown_offset + stroke * 0.14;
    let cyl_bot_dist = l_rod - r_crank - piston_h * 0.26;
    let mid_dist = 0.5 * (head_dist + cyl_bot_dist);
    let trumpet_len_m = (snap.trumpet_length_mm as f32) * 0.001;
    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);

    // 1. Cast Billet Dry-Sump Oil Pan & Lower Bedplate (segmented per bay for accurate Z-depth sorting)
    for bay in 0..5 {
        let zb = (bay as f32 - 2.0) * throw_spacing;
        scene.push_box_3d(
            cam,
            [0.0, -0.044, zb],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.132,
            0.018,
            bay_hz,
            Color32::from_rgb(24, 28, 36),
        );
    }

    // Front & Rear Timing Case Bulkhead Plates
    for &z_plate in &[z_front, z_rear] {
        scene.push_box_3d(
            cam,
            [0.0, 0.042, z_plate],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.125,
            0.068,
            0.010,
            Color32::from_rgb(32, 37, 47),
        );
    }

    // Main Crankshaft Core Journal
    scene.push_cylinder_3d(
        cam,
        [0.0, 0.0, z_front - 0.042],
        [0.0, 0.0, z_rear + 0.042],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.018,
        0.018,
        10,
        Color32::from_rgb(104, 114, 132),
        false,
        true,
    );

    // 2. Lower Block Thermal Pre-Heater Manifolds (Click Step 1)
    let mut block_col = if min_temp >= 60.0 {
        Color32::from_rgb(56, 136, 92)
    } else if ctrl.preheat_on() {
        Color32::from_rgb(208, 132, 46)
    } else {
        Color32::from_rgb(56, 63, 76)
    };
    if highlight == EnginePart::CylinderBlock {
        block_col = tint_mix(block_col, ACCENT_AMBER, 0.45);
    }
    let mut preheat_pts = Vec::new();
    for &sx in &[-1.0_f32, 1.0_f32] {
        let hx = sx * 0.145;
        let hy = 0.036;
        for bay in 0..5 {
            let zb = (bay as f32 - 2.0) * throw_spacing;
            scene.push_box_3d(
                cam,
                [hx, hy, zb],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                0.014,
                0.016,
                bay_hz,
                block_col,
            );
            if let Some((p2, _, _)) = cam.project([hx, hy, zb]) {
                preheat_pts.push(p2);
            }
        }
    }

    // 3. Sculpted Bank Cylinder Head Castings + Pneumatic Cam Covers (Click Step 2)
    let mut pneu_col = if ctrl.n2_pressure_bar >= 100.0 {
        Color32::from_rgb(56, 112, 162)
    } else {
        Color32::from_rgb(46, 53, 66)
    };
    if highlight == EnginePart::PneumaticHeads {
        pneu_col = tint_mix(pneu_col, ACCENT_CYAN, 0.45);
    }
    let mut n2_pts = Vec::new();
    for &bank_ang in &[-half_v, half_v] {
        let u_axis = [bank_ang.sin(), bank_ang.cos(), 0.0];
        let u_perp = [bank_ang.cos(), -bank_ang.sin(), 0.0];
        let u_z = [0.0, 0.0, 1.0];

        let in_sign = if bank_ang < 0.0 { 1.0 } else { -1.0 };
        let valley_wall_base = add3(
            mul3(u_axis, mid_dist),
            mul3(u_perp, in_sign * bore * 0.56),
        );
        let cover_base = mul3(u_axis, head_dist + bore * 0.37);
        for bay in 0..5 {
            let zb = (bay as f32 - 2.0) * throw_spacing;
            scene.push_box_3d(
                cam,
                [valley_wall_base[0], valley_wall_base[1], zb],
                u_perp,
                u_axis,
                u_z,
                0.010,
                0.5 * (head_dist - cyl_bot_dist),
                bay_hz,
                Color32::from_rgb(30, 35, 44),
            );
            scene.push_box_3d(
                cam,
                [cover_base[0], cover_base[1], zb],
                u_perp,
                u_axis,
                u_z,
                bore * 0.48,
                0.016,
                bay_hz,
                pneu_col,
            );
            if let Some((p2, _, _)) = cam.project([cover_base[0], cover_base[1], zb]) {
                n2_pts.push(p2);
            }
        }
    }

    // 4. Longitudinal 5-into-1 Collector Manifolds & Rear Tails
    let ex_col = header_glow_color(snap.egt_c);
    let col_z_a = z_front + 0.04;
    let col_z_b = z_rear + 0.04;
    for &sx in &[-1.0_f32, 1.0_f32] {
        let col_x = sx * 0.258;
        let col_y = 0.128;
        scene.push_cylinder_3d(
            cam,
            [col_x, col_y, col_z_a],
            [col_x, col_y, col_z_b],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            0.019,
            0.026,
            10,
            ex_col,
            false,
            true,
        );
        scene.push_cylinder_3d(
            cam,
            [col_x, col_y, col_z_b],
            [col_x * 0.96, col_y - 0.010, col_z_b + 0.135],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            0.026,
            0.029,
            12,
            ex_col,
            false,
            true,
        );
    }

    // 5. All 10 Cylinders, Pistons, Crank Throws, Conrods, Valves, Trumpets & Primary Headers
    let mut trumpet_pts = Vec::new();
    for throw_idx in 0..5 {
        let zt = (throw_idx as f32 - 2.0) * throw_spacing;
        let cyl_l_idx = throw_idx;
        let cyl_r_idx = throw_idx + 5;

        let pin_angle = (snap.cylinders[cyl_l_idx].local_angle_rad as f32) - half_v;
        let pin_dir = [pin_angle.sin(), pin_angle.cos(), 0.0];
        let pin_perp = [pin_angle.cos(), -pin_angle.sin(), 0.0];
        let pin_pos = [pin_dir[0] * r_crank, pin_dir[1] * r_crank, zt];

        // True 3D Counterweight Web (cylindrical sector opposite crankpin — never billboarded!)
        let cw_c = [-pin_dir[0] * r_crank * 0.55, -pin_dir[1] * r_crank * 0.55, zt];
        scene.push_box_3d(
            cam,
            cw_c,
            pin_perp,
            pin_dir,
            [0.0, 0.0, 1.0],
            r_crank * 1.15,
            r_crank * 0.95,
            0.011,
            Color32::from_rgb(54, 60, 72),
        );
        scene.push_rod(cam, [0.0, 0.0, zt], pin_pos, 0.009, Color32::from_rgb(138, 148, 165));
        scene.push_cylinder_3d(
            cam,
            [pin_pos[0], pin_pos[1], zt - 0.018],
            [pin_pos[0], pin_pos[1], zt + 0.018],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            0.0085,
            0.0085,
            8,
            ACCENT_AMBER,
            true,
            true,
        );

        for &(cyl_idx, bank_ang, ex_bank_x) in &[
            (cyl_l_idx, -half_v, -0.258_f32),
            (cyl_r_idx, half_v, 0.258_f32),
        ] {
            let cyl = &snap.cylinders[cyl_idx];
            let z_cyl = zt + if bank_ang < 0.0 { -0.012 } else { 0.012 };
            let u_axis = [bank_ang.sin(), bank_ang.cos(), 0.0];
            let u_perp = [bank_ang.cos(), -bank_ang.sin(), 0.0];
            let u_z = [0.0, 0.0, 1.0];

            // Kinematic slider-crank wrist pin
            let d = [pin_pos[0], pin_pos[1], 0.0];
            let proj = dot3(d, u_axis);
            let perp = dot3(d, u_perp);
            let wrist_dist = proj + (l_rod * l_rod - perp * perp).max(0.0001).sqrt();
            let wrist_pos = add3([0.0, 0.0, z_cyl], mul3(u_axis, wrist_dist));

            // Titanium Conrod
            scene.push_rod(
                cam,
                [pin_pos[0], pin_pos[1], z_cyl],
                wrist_pos,
                0.0068,
                Color32::from_rgb(144, 154, 170),
            );

            // Low-Poly 12-Facet Forged Piston
            let crown_dist = wrist_dist + crown_offset;
            let skirt_dist = wrist_dist - piston_h * 0.45;
            scene.push_cylinder_3d(
                cam,
                add3([0.0, 0.0, z_cyl], mul3(u_axis, skirt_dist)),
                add3([0.0, 0.0, z_cyl], mul3(u_axis, crown_dist)),
                u_perp,
                u_z,
                bore * 0.45,
                bore * 0.45,
                12,
                Color32::from_rgb(172, 180, 196),
                true,
                true,
            );

            // Combustion Gas Charge Cylinder
            if head_dist > crown_dist + 0.0015 {
                let g_col = gas_color(cyl);
                scene.push_cylinder_3d(
                    cam,
                    add3([0.0, 0.0, z_cyl], mul3(u_axis, crown_dist)),
                    add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist)),
                    u_perp,
                    u_z,
                    bore * 0.43,
                    bore * 0.43,
                    12,
                    g_col,
                    false,
                    false,
                );
            }

            // Cutaway Translucent Cylinder Sleeve (2 axial bands for clean depth sorting around pistons)
            let sleeve_col = if highlight == EnginePart::CylinderBlock {
                Color32::from_rgba_unmultiplied(210, 148, 68, 52)
            } else {
                Color32::from_rgba_unmultiplied(62, 74, 94, 34)
            };
            let sleeve_c0 = add3([0.0, 0.0, z_cyl], mul3(u_axis, cyl_bot_dist));
            let sleeve_mid = add3([0.0, 0.0, z_cyl], mul3(u_axis, mid_dist));
            let sleeve_c1 = add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist));
            scene.push_cylinder_3d(cam, sleeve_c0, sleeve_mid, u_perp, u_z, bore * 0.49, bore * 0.49, 12, sleeve_col, false, false);
            scene.push_cylinder_3d(cam, sleeve_mid, sleeve_c1, u_perp, u_z, bore * 0.49, bore * 0.49, 12, sleeve_col, false, false);

            let head_c = add3([0.0, 0.0, z_cyl], mul3(u_axis, head_dist));
            let block_mid = add3([0.0, 0.0, z_cyl], mul3(u_axis, mid_dist));
            if let Some((p2, _, _)) = cam.project(block_mid) {
                preheat_pts.push(p2);
            }

            // Valves (2 intake + 2 exhaust)
            let in_sign = if bank_ang < 0.0 { 1.0 } else { -1.0 };
            let ex_sign = -in_sign;
            let in_port = add3(head_c, mul3(u_perp, in_sign * bore * 0.22));
            let ex_port = add3(head_c, mul3(u_perp, ex_sign * bore * 0.22));

            let in_travel = (cyl.intake_lift_norm as f32) * (bore * 0.14);
            let ex_travel = (cyl.exhaust_lift_norm as f32) * (bore * 0.14);
            for &vz in &[-bore * 0.16, bore * 0.16] {
                let seat_in = add3(add3(in_port, [0.0, 0.0, vz]), mul3(u_axis, -in_travel));
                let stem_in = add3(seat_in, mul3(u_axis, bore * 0.30));
                scene.push_rod(cam, seat_in, stem_in, 0.0024, Color32::from_rgb(130, 200, 245));

                let seat_ex = add3(add3(ex_port, [0.0, 0.0, vz]), mul3(u_axis, -ex_travel));
                let stem_ex = add3(seat_ex, mul3(u_axis, bore * 0.30));
                scene.push_rod(cam, seat_ex, stem_ex, 0.0024, Color32::from_rgb(240, 140, 95));
            }

            if cyl.spark_firing {
                scene.push_spark(cam, head_c, 0.015, Color32::from_rgb(130, 232, 255));
                scene.push_spark(cam, head_c, 0.007, Color32::WHITE);
            }

            // Variable-Length Flared Intake Velocity Stack
            let stack_col = if highlight == EnginePart::IntakeTrumpets {
                Color32::from_rgb(86, 102, 128)
            } else {
                Color32::from_rgb(56, 64, 78)
            };
            let runner_base = add3(in_port, mul3(u_axis, 0.016));
            let stack_dir = norm3(add3(mul3(u_axis, 0.92), mul3(u_perp, in_sign * 0.26)));
            let trumpet_top = add3(runner_base, mul3(stack_dir, 0.030 + trumpet_len_m));
            scene.push_cylinder_3d(
                cam,
                runner_base,
                trumpet_top,
                u_perp,
                u_z,
                bore * 0.20,
                bore * 0.26,
                12,
                stack_col,
                false,
                true,
            );
            scene.push_disc_3d(
                cam,
                trumpet_top,
                u_perp,
                u_z,
                bore * 0.23,
                12,
                Color32::from_rgb(12, 14, 18),
                false,
            );

            let tps_ang = (snap.effective_throttle as f32) * 1.48;
            let butterfly_c = add3(runner_base, mul3(stack_dir, 0.025));
            let plate_dir = add3(mul3(u_perp, tps_ang.cos()), mul3(stack_dir, tps_ang.sin()));
            scene.push_line_3d(
                cam,
                sub3(butterfly_c, mul3(plate_dir, bore * 0.16)),
                add3(butterfly_c, mul3(plate_dir, bore * 0.16)),
                Stroke::new(2.4_f32, ACCENT_AMBER),
            );
            if cyl.fuel_injecting {
                scene.push_line_3d(
                    cam,
                    add3(runner_base, mul3(stack_dir, 0.018)),
                    add3(in_port, mul3(u_axis, -0.010)),
                    Stroke::new(2.2_f32, Color32::from_rgba_unmultiplied(92, 180, 230, 175)),
                );
            }
            if let Some((p2, _, _)) = cam.project(trumpet_top) {
                trumpet_pts.push(p2);
            }

            // Clean Sculpted Exhaust Primary -> Outer Collector Log (zero overlapping spiderweb lines!)
            let p_mid = add3(add3(ex_port, mul3(u_axis, 0.014)), mul3(u_perp, ex_sign * 0.052));
            let p_col = [ex_bank_x, 0.128, (z_cyl + 0.025).clamp(col_z_a, col_z_b)];
            scene.push_rod(cam, ex_port, p_mid, 0.013, ex_col);
            scene.push_rod(cam, p_mid, p_col, 0.013, ex_col);
        }
    }

    // 6. Central V-Valley High-Pressure Fuel Rail & Ignition Pack (Click Step 3)
    let mut rail_col = if ctrl.armed() {
        ACCENT_GREEN
    } else {
        Color32::from_rgb(76, 84, 98)
    };
    if highlight == EnginePart::SparkPlugs {
        rail_col = tint_mix(rail_col, ACCENT_GREEN, 0.50);
    }
    let rail_y = head_dist * 0.94 + 0.042;
    let mut rail_pts = Vec::new();
    for t in 0..5 {
        let zt = (t as f32 - 2.0) * throw_spacing;
        scene.push_box_3d(
            cam,
            [0.0, rail_y, zt],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.014,
            0.012,
            bay_hz,
            rail_col,
        );
        let cp = [0.0, rail_y + 0.008, zt];
        scene.push_box_3d(
            cam,
            cp,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            0.018,
            0.010,
            0.016,
            rail_col,
        );
        if let Some((p2, _, _)) = cam.project(cp) {
            rail_pts.push(p2);
        }
    }

    // 7. Side High-Pressure N2 Pneumatic Accumulator Bottle (Click Step 2)
    let bot_a = [0.232_f32, 0.026_f32, -0.09_f32];
    let bot_b = [0.232_f32, 0.026_f32, 0.17_f32];
    let mut bottle_col = if ctrl.n2_pressure_bar >= 100.0 {
        ACCENT_CYAN
    } else {
        Color32::from_rgb(58, 70, 88)
    };
    if highlight == EnginePart::PneumaticHeads {
        bottle_col = tint_mix(bottle_col, ACCENT_CYAN, 0.50);
    }
    scene.push_cylinder_3d(
        cam,
        bot_a,
        bot_b,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.032,
        0.032,
        12,
        bottle_col,
        true,
        true,
    );
    for &bp in &[bot_a, [0.232, 0.026, 0.04], bot_b] {
        if let Some((p2, _, _)) = cam.project(bp) {
            n2_pts.push(p2);
        }
    }

    // 8. Front Timing Damper Pulley (Drag when stopped to hand-crank)
    let front_c0 = [0.0, 0.0, z_front - 0.030];
    let front_c1 = [0.0, 0.0, z_front - 0.048];
    let pulley_r = r_crank * 2.55;
    let pulley_col = if highlight == EnginePart::FlywheelHandCrank {
        Color32::from_rgb(66, 76, 94)
    } else {
        Color32::from_rgb(42, 48, 58)
    };
    scene.push_cylinder_3d(
        cam,
        front_c0,
        front_c1,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        pulley_r,
        pulley_r,
        16,
        pulley_col,
        true,
        true,
    );
    let ca = snap.crank_angle_rad as f32;
    scene.push_line_3d(
        cam,
        [0.0, 0.0, front_c1[2] - 0.003],
        [ca.sin() * pulley_r * 0.85, ca.cos() * pulley_r * 0.85, front_c1[2] - 0.003],
        Stroke::new(3.0_f32, ACCENT_AMBER),
    );
    let front_pulley_pos = cam.project(front_c1).map(|(p, _, _)| p).unwrap_or(Pos2::ZERO);

    // 9. Rear Carbon Flywheel + External Pneumatic Starter Drive (Click & Hold Step 4)
    let fw_c0 = [0.0, 0.0, z_rear + 0.028];
    let fw_c1 = [0.0, 0.0, z_rear + 0.052];
    let fw_r = r_crank * 3.85;
    scene.push_cylinder_3d(
        cam,
        fw_c0,
        fw_c1,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        fw_r,
        fw_r,
        16,
        Color32::from_rgb(30, 34, 42),
        true,
        true,
    );
    scene.push_line_3d(
        cam,
        [ca.sin() * (-fw_r * 0.85), ca.cos() * (-fw_r * 0.85), fw_c1[2] + 0.003],
        [ca.sin() * (fw_r * 0.85), ca.cos() * (fw_r * 0.85), fw_c1[2] + 0.003],
        Stroke::new(2.4_f32, Color32::from_rgb(122, 132, 150)),
    );

    let ready_to_crank = min_temp >= 60.0 && ctrl.n2_pressure_bar >= 100.0 && ctrl.armed() && !snap.is_running;
    let mut starter_col = if ctrl.starter_active {
        ACCENT_AMBER
    } else if ready_to_crank {
        ACCENT_GREEN
    } else {
        Color32::from_rgb(88, 96, 112)
    };
    if highlight == EnginePart::CrankHubStarter {
        starter_col = tint_mix(starter_col, ACCENT_AMBER, 0.45);
    }
    let starter_a = [0.0, 0.0, z_rear + 0.054];
    let starter_b = [0.0, 0.0, z_rear + 0.165];
    scene.push_cylinder_3d(
        cam,
        starter_a,
        starter_b,
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0.032,
        0.035,
        12,
        starter_col,
        true,
        true,
    );
    let starter_pos = cam.project(starter_b).map(|(p, _, _)| p).unwrap_or(Pos2::ZERO);

    HitTargets3D {
        preheat_pts,
        n2_pts,
        rail_pts,
        starter_pos,
        front_pulley_pos,
        trumpet_pts,
    }
}

// ============================================================================
// OPEN-SPACE MINIMAL HUD ELEMENTS
// ============================================================================

fn draw_top_hud(
    ui: &mut egui::Ui,
    p: &egui::Painter,
    full_rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &SharedControls,
) {
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
            Color32::from_rgb(20, 24, 31)
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
        p.text(Pos2::new(*x, led_y + 13.0), Align2::CENTER_TOP, *lbl, FontId::proportional(10.0), TEXT_MUTED);
        p.text(Pos2::new(*x, led_y + 25.0), Align2::CENTER_TOP, val, FontId::proportional(17.5), *c);
    }

    // Subtle status badge top-right
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
        ("WARMING FLUIDS", ACCENT_AMBER)
    } else {
        ("OFFLINE", TEXT_MUTED)
    };
    p.text(
        Pos2::new(full_rect.right() - 22.0, full_rect.top() + 22.0),
        Align2::RIGHT_TOP,
        st_txt,
        FontId::proportional(12.5),
        st_col,
    );

    // Top-left minimal engine title
    p.text(
        Pos2::new(full_rect.left() + 26.0, full_rect.top() + 20.0),
        Align2::LEFT_TOP,
        "F1 3.0L V10",
        FontId::proportional(15.0),
        TEXT_PRIMARY,
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
    p.text(
        Pos2::new(bar_rect.left(), bar_rect.top() - 16.0),
        Align2::LEFT_BOTTOM,
        format!("THROTTLE {:3.0}%", ctrl.throttle * 100.0),
        FontId::proportional(11.5),
        TEXT_PRIMARY,
    );

    let resp = ui.interact(hit_rect, ui.id().with("vert_throttle"), Sense::click_and_drag());
    if resp.drag_started() || resp.clicked() || resp.is_pointer_button_down_on() {
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

    p.rect_filled(bar_rect, 3.0, BG_PILL);
    p.rect_stroke(bar_rect, 3.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let req_h = (ctrl.throttle as f32) * bar_rect.height();
    let eff_h = (snap.effective_throttle as f32) * bar_rect.height();
    if eff_h > 1.0 {
        let eff_rect = Rect::from_min_max(
            Pos2::new(bar_rect.left() + 2.0, bar_rect.bottom() - eff_h),
            Pos2::new(bar_rect.right() - 2.0, bar_rect.bottom()),
        );
        p.rect_filled(eff_rect, 2.0, Color32::from_rgba_unmultiplied(92, 180, 230, 50));
    }
    if req_h > 1.0 {
        let req_rect = Rect::from_min_max(
            Pos2::new(bar_rect.left() + 3.0, bar_rect.bottom() - req_h),
            Pos2::new(bar_rect.right() - 3.0, bar_rect.bottom() - 2.0),
        );
        p.rect_filled(req_rect, 2.0, ACCENT_GREEN);
    }

    let handle_y = bar_rect.bottom() - req_h;
    p.line_segment(
        [Pos2::new(bar_rect.left() - 3.0, handle_y), Pos2::new(bar_rect.right() + 3.0, handle_y)],
        Stroke::new(2.5_f32, TEXT_PRIMARY),
    );

    let marks = [
        (1.00_f32, "100%  [SPACE]", ACCENT_GREEN),
        (0.75_f32, "75%   [W / ↑]", TEXT_MUTED),
        (0.50_f32, "50%", TEXT_MUTED),
        (0.25_f32, "25%   [S / ↓]", TEXT_MUTED),
        (0.00_f32, "0%    Idle", TEXT_MUTED),
    ];
    for &(frac, label, col) in &marks {
        let ty = bar_rect.bottom() - frac * bar_rect.height();
        p.line_segment(
            [Pos2::new(bar_rect.right() + 2.0, ty), Pos2::new(bar_rect.right() + 9.0, ty)],
            Stroke::new(1.0_f32, BORDER_SUBTLE),
        );
        p.text(
            Pos2::new(bar_rect.right() + 13.0, ty),
            Align2::LEFT_CENTER,
            label,
            FontId::proportional(10.5),
            col,
        );
    }

    p.text(
        Pos2::new(bar_rect.left(), bar_rect.bottom() + 18.0),
        Align2::LEFT_TOP,
        "[Z/X] Shift  •  [Q] Auto  •  [ESC] Reset",
        FontId::proportional(10.5),
        Color32::from_rgb(102, 110, 125),
    );
}

#[allow(clippy::too_many_arguments)]
fn draw_bottom_hamburger_menu(
    ui: &mut egui::Ui,
    p: &egui::Painter,
    btn_rect: Rect,
    popover_rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    menu_open: &mut bool,
    cam_yaw: &mut f32,
    cam_pitch: &mut f32,
    cam_zoom: &mut f32,
) {
    let resp = ui.interact(btn_rect, ui.id().with("hamburger_btn"), Sense::click());
    if resp.clicked() {
        *menu_open = !*menu_open;
    }
    let fill = if *menu_open || resp.hovered() {
        Color32::from_rgb(24, 28, 37)
    } else {
        BG_PILL
    };
    p.rect_filled(btn_rect, 4.0, fill);
    p.rect_stroke(btn_rect, 4.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let icon_x0 = btn_rect.left() + 11.0;
    let icon_x1 = btn_rect.left() + 25.0;
    let cy = btn_rect.center().y;
    for &dy in &[-4.5_f32, 0.0_f32, 4.5_f32] {
        p.line_segment(
            [Pos2::new(icon_x0, cy + dy), Pos2::new(icon_x1, cy + dy)],
            Stroke::new(1.6_f32, TEXT_PRIMARY),
        );
    }

    let mode_text = match ctrl.bench_mode {
        BenchMode::FreeRev => "MANUAL REV".to_string(),
        BenchMode::DynoSweep => {
            if snap.dyno_sweep_running {
                "AUTO PULL...".to_string()
            } else {
                "AUTO PULL".to_string()
            }
        }
        BenchMode::MonzaHotlapAuto => format!("MONZA • {}", snap.monza_sector_name),
        BenchMode::TrackManualGearbox => format!("GEAR {} [Z/X]", ctrl.requested_gear),
        BenchMode::DynoManualBrake | BenchMode::DynoRpmHold => "DYNO BENCH".to_string(),
    };
    p.text(
        Pos2::new(icon_x1 + 10.0, cy),
        Align2::LEFT_CENTER,
        mode_text,
        FontId::proportional(11.0),
        ACCENT_CYAN,
    );

    if !*menu_open {
        return;
    }

    p.rect_filled(popover_rect, 5.0, bg_popover());
    p.rect_stroke(popover_rect, 5.0, Stroke::new(1.0_f32, Color32::from_rgb(44, 52, 66)));

    ui.allocate_new_ui(
        egui::UiBuilder::new().max_rect(popover_rect.shrink(10.0)),
        |ui| {
            ui.label(RichText::new("DRIVE / TEST MODE").size(10.0).color(TEXT_MUTED));
            let mode_items = [
                (BenchMode::FreeRev, "Manual Rev"),
                (BenchMode::DynoSweep, "Auto Pull (5.5k–19.5k)"),
                (BenchMode::MonzaHotlapAuto, "Monza Lap (Auto)"),
                (BenchMode::TrackManualGearbox, "7-Spd Gearbox [Z/X]"),
            ];
            for &(m, label) in &mode_items {
                let sel = ctrl.bench_mode == m;
                if ui
                    .add_sized([220.0, 22.0], egui::SelectableLabel::new(sel, label))
                    .clicked()
                {
                    ctrl.bench_mode = m;
                    if m == BenchMode::DynoSweep {
                        ctrl.start_dyno_sweep = true;
                    } else if m == BenchMode::MonzaHotlapAuto && !snap.is_running {
                        ctrl.quick_start_trigger = true;
                    }
                    *menu_open = false;
                }
            }

            ui.separator();

            ui.horizontal(|ui| {
                if ui.small_button("Auto Start [Q]").clicked() {
                    ctrl.quick_start_trigger = true;
                    *menu_open = false;
                }
                if ui.small_button("Reset [ESC]").clicked() {
                    ctrl.reset_repair_trigger = true;
                    *cam_yaw = -0.62;
                    *cam_pitch = 0.34;
                    *cam_zoom = 1.0;
                    *menu_open = false;
                }
                if ui.small_button("View [R]").clicked() {
                    *cam_yaw = -0.62;
                    *cam_pitch = 0.34;
                    *cam_zoom = 1.0;
                    *menu_open = false;
                }
            });

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                let mute_lbl = if ctrl.muted { "Muted" } else { "Audio" };
                if ui.small_button(mute_lbl).clicked() {
                    ctrl.muted = !ctrl.muted;
                }
                ui.add(egui::Slider::new(&mut ctrl.master_volume, 0.0..=1.5).show_value(false));
            });
        },
    );
}

fn draw_footer_prompt(
    p: &egui::Painter,
    center_x: f32,
    y: f32,
    snap: &TelemetrySnapshot,
    ctrl: &SharedControls,
    highlight: EnginePart,
) {
    let min_temp = snap.coolant_temp_c.min(snap.oil_temp_c);
    let warm_ok = min_temp >= 60.0;
    let pneu_ok = ctrl.n2_pressure_bar >= 100.0;
    let armed_ok = ctrl.armed();

    let (msg, col) = if highlight != EnginePart::None {
        let m = match highlight {
            EnginePart::CylinderBlock => "3D Cylinder Block Pre-Heater — Click to toggle Pre-Heat (22°C -> 60°C)",
            EnginePart::PneumaticHeads => "3D N2 Accumulator & Cam Covers — Click to pressurize valve springs (165 bar)",
            EnginePart::SparkPlugs => "3D V-Valley Fuel Rail & Coils — Click to arm Fuel & Ignition",
            EnginePart::CrankHubStarter => "3D Rear Pneumatic Air Starter — Click & Hold to crank V10",
            EnginePart::IntakeTrumpets => "3D Intake Trumpets — Drag Up / Down to open throttle",
            EnginePart::FlywheelHandCrank => "3D Front Crank Pulley — Drag circularly when stopped to hand-crank",
            EnginePart::None => "",
        };
        (m.to_string(), TEXT_PRIMARY)
    } else if snap.cold_seized {
        ("COLD SEIZED (<60°C) — Click the 3D Block Pre-Heaters to warm fluids first".to_string(), ACCENT_RED)
    } else if !warm_ok && !ctrl.preheat_on() {
        (
            format!("STEP 1/4: Click the 3D Lower Block Manifolds to Pre-Heat ({:.0}°C -> 60°C)  •  or [Q] Auto Start", min_temp),
            ACCENT_AMBER,
        )
    } else if !warm_ok {
        (format!("Warming block & oil... ({:.0}°C / 60°C)", min_temp), ACCENT_AMBER)
    } else if !pneu_ok {
        ("STEP 2/4: Click the 3D N2 Cylinder (right flank) or Cam Covers to pressurize (165 bar)".to_string(), ACCENT_AMBER)
    } else if !armed_ok {
        ("STEP 3/4: Click the 3D V-Valley Fuel Rail (center top) to arm Fuel & Ignition".to_string(), ACCENT_AMBER)
    } else if !snap.is_running {
        ("STEP 4/4: Click & Hold the 3D Rear Air Starter to fire up the V10".to_string(), ACCENT_GREEN)
    } else {
        ("MMB Drag: Orbit 3D Engine  •  Scroll: Zoom  •  [ESC] Reset  •  [R] Reset View".to_string(), TEXT_MUTED)
    };

    p.text(Pos2::new(center_x, y), Align2::CENTER_BOTTOM, msg, FontId::proportional(11.0), col);
}

fn draw_right_hud(
    ui: &mut egui::Ui,
    rect: Rect,
    snap: &TelemetrySnapshot,
    ctrl: &mut SharedControls,
    diag_plot: &mut DiagnosticPlot,
) {
    ui.allocate_new_ui(egui::UiBuilder::new().max_rect(rect), |ui| {
        draw_dyno_card(ui, snap, ctrl);
        ui.add_space(10.0);
        draw_diag_card(ui, snap, diag_plot);
        ui.add_space(10.0);
        draw_vitals_strip(ui, snap, ctrl);
    });
}

#[inline]
fn catmull_rom_scalar(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * ((2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3)
}

fn draw_dyno_card(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &mut SharedControls) {
    let card_h = 232.0_f32;
    let card_w = ui.available_width().max(290.0);
    let (card_rect, _) = ui.allocate_exact_size(Vec2::new(card_w, card_h), Sense::hover());
    let p = ui.painter_at(card_rect);
    p.rect_filled(card_rect, 5.0, bg_card());
    p.rect_stroke(card_rect, 5.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let (peak_bhp, peak_bhp_rpm, peak_nm, peak_nm_rpm) = compute_dyno_peaks(snap);
    p.text(
        Pos2::new(card_rect.left() + 10.0, card_rect.top() + 8.0),
        Align2::LEFT_TOP,
        "DYNO",
        FontId::proportional(11.0),
        TEXT_MUTED,
    );

    // Quick [PULL] and [CLR] actions in dyno card header
    let pull_rect = Rect::from_min_size(Pos2::new(card_rect.right() - 78.0, card_rect.top() + 5.0), Vec2::new(36.0, 17.0));
    let clr_rect = Rect::from_min_size(Pos2::new(card_rect.right() - 38.0, card_rect.top() + 5.0), Vec2::new(30.0, 17.0));
    let pull_resp = ui.interact(pull_rect, ui.id().with("dyno_pull"), Sense::click());
    let clr_resp = ui.interact(clr_rect, ui.id().with("dyno_clr"), Sense::click());
    if pull_resp.clicked() {
        ctrl.bench_mode = BenchMode::DynoSweep;
        ctrl.start_dyno_sweep = true;
    }
    if clr_resp.clicked() {
        ctrl.clear_dyno_trace = true;
    }
    p.rect_filled(
        pull_rect,
        3.0,
        if snap.dyno_sweep_running {
            Color32::from_rgb(28, 70, 48)
        } else if pull_resp.hovered() {
            Color32::from_rgb(28, 36, 46)
        } else {
            BG_PILL
        },
    );
    p.text(
        pull_rect.center(),
        Align2::CENTER_CENTER,
        if snap.dyno_sweep_running { "RUN" } else { "PULL" },
        FontId::proportional(9.5),
        if snap.dyno_sweep_running || pull_resp.hovered() { ACCENT_GREEN } else { TEXT_PRIMARY },
    );
    p.text(
        clr_rect.center(),
        Align2::CENTER_CENTER,
        "CLR",
        FontId::proportional(9.5),
        if clr_resp.hovered() { ACCENT_RED } else { TEXT_MUTED },
    );

    // Dual Y-axes: Left Green (Power 0..1000 BHP), Right Amber (Torque 0..500 Nm)
    let pad = Rect::from_min_max(
        Pos2::new(card_rect.left() + 34.0, card_rect.top() + 34.0),
        Pos2::new(card_rect.right() - 34.0, card_rect.bottom() - 24.0),
    );
    let max_bhp = 1000.0_f32;
    let max_nm = 500.0_f32;
    let rpm_span = (DYNO_MAX_RPM - DYNO_MIN_RPM) as f32;

    for &(frac, bhp_lbl, nm_lbl) in &[
        (0.25_f32, "250", "125"),
        (0.50_f32, "500", "250"),
        (0.75_f32, "750", "375"),
        (1.00_f32, "1000", "500"),
    ] {
        let y = pad.bottom() - frac * pad.height();
        p.line_segment(
            [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
            Stroke::new(0.6_f32, Color32::from_rgb(20, 24, 32)),
        );
        p.text(Pos2::new(pad.left() - 4.0, y), Align2::RIGHT_CENTER, bhp_lbl, FontId::proportional(9.0), ACCENT_GREEN);
        p.text(Pos2::new(pad.right() + 4.0, y), Align2::LEFT_CENTER, nm_lbl, FontId::proportional(9.0), ACCENT_AMBER);
    }

    for &(rpm_tick, lbl) in &[
        (6000.0, "6k"),
        (8000.0, "8k"),
        (10000.0, "10k"),
        (12000.0, "12k"),
        (14000.0, "14k"),
        (16000.0, "16k"),
        (18000.0, "18k"),
        (20000.0, "20k"),
    ] {
        let nx = ((rpm_tick - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
        let x = pad.left() + nx * pad.width();
        p.line_segment(
            [Pos2::new(x, pad.top()), Pos2::new(x, pad.bottom())],
            Stroke::new(0.6_f32, Color32::from_rgb(20, 24, 32)),
        );
        p.text(Pos2::new(x, pad.bottom() + 5.0), Align2::CENTER_TOP, lbl, FontId::proportional(9.0), TEXT_MUTED);
    }

    // Gather smoothed recorded data samples (rpm, bhp, nm)
    let mut rec_samples: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..DYNO_BINS {
        let pt = snap.dyno_curve[i];
        if pt.recorded {
            let mut sum_p = pt.power_bhp;
            let mut sum_t = pt.torque_nm;
            let mut w = 1.0_f32;
            if i > 0 && snap.dyno_curve[i - 1].recorded {
                sum_p += 0.5 * snap.dyno_curve[i - 1].power_bhp;
                sum_t += 0.5 * snap.dyno_curve[i - 1].torque_nm;
                w += 0.5;
            }
            if i + 1 < DYNO_BINS && snap.dyno_curve[i + 1].recorded {
                sum_p += 0.5 * snap.dyno_curve[i + 1].power_bhp;
                sum_t += 0.5 * snap.dyno_curve[i + 1].torque_nm;
                w += 0.5;
            }
            rec_samples.push((pt.rpm, (sum_p / w).max(0.0), (sum_t / w).max(0.0)));
        }
    }

    // Build smooth Catmull-Rom curve points
    let mut spline_data: Vec<(f32, f32, f32)> = Vec::new();
    let mut bhp_pts: Vec<Pos2> = Vec::new();
    let mut nm_pts: Vec<Pos2> = Vec::new();
    if rec_samples.len() >= 2 {
        let n = rec_samples.len();
        for i in 0..(n - 1) {
            let s0 = rec_samples[i.saturating_sub(1)];
            let s1 = rec_samples[i];
            let s2 = rec_samples[i + 1];
            let s3 = rec_samples[(i + 2).min(n - 1)];
            let sub_steps = if i + 2 == n { 5 } else { 4 };
            let limit = if i + 2 == n { sub_steps } else { sub_steps - 1 };
            for step in 0..=limit {
                let t = step as f32 / 4.0;
                let rpm = catmull_rom_scalar(s0.0, s1.0, s2.0, s3.0, t).clamp(DYNO_MIN_RPM as f32, DYNO_MAX_RPM as f32);
                let bhp = catmull_rom_scalar(s0.1, s1.1, s2.1, s3.1, t).clamp(0.0, max_bhp);
                let nm = catmull_rom_scalar(s0.2, s1.2, s2.2, s3.2, t).clamp(0.0, max_nm);
                let x = pad.left() + ((rpm - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0) * pad.width();
                let y_bhp = pad.bottom() - (bhp / max_bhp).clamp(0.0, 1.0) * pad.height();
                let y_nm = pad.bottom() - (nm / max_nm).clamp(0.0, 1.0) * pad.height();
                spline_data.push((rpm, bhp, nm));
                bhp_pts.push(Pos2::new(x, y_bhp));
                nm_pts.push(Pos2::new(x, y_nm));
            }
        }
    }

    if bhp_pts.len() >= 2 {
        let bhp_area = Color32::from_rgba_unmultiplied(72, 188, 120, 26);
        let nm_area = Color32::from_rgba_unmultiplied(222, 158, 62, 14);
        for w in nm_pts.windows(2) {
            if (w[1].x - w[0].x).abs() > 0.4 && (pad.bottom() - w[0].y).max(pad.bottom() - w[1].y) > 0.4 {
                p.add(egui::Shape::convex_polygon(
                    vec![w[0], w[1], Pos2::new(w[1].x, pad.bottom()), Pos2::new(w[0].x, pad.bottom())],
                    nm_area,
                    Stroke::NONE,
                ));
            }
        }
        for w in bhp_pts.windows(2) {
            if (w[1].x - w[0].x).abs() > 0.4 && (pad.bottom() - w[0].y).max(pad.bottom() - w[1].y) > 0.4 {
                p.add(egui::Shape::convex_polygon(
                    vec![w[0], w[1], Pos2::new(w[1].x, pad.bottom()), Pos2::new(w[0].x, pad.bottom())],
                    bhp_area,
                    Stroke::NONE,
                ));
            }
        }
        p.add(egui::Shape::line(nm_pts, Stroke::new(1.7_f32, ACCENT_AMBER)));
        p.add(egui::Shape::line(bhp_pts, Stroke::new(2.2_f32, ACCENT_GREEN)));

        // Peak ring markers on curve
        if peak_bhp > 1.0 {
            let px = pad.left() + ((peak_bhp_rpm - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0) * pad.width();
            let py = pad.bottom() - (peak_bhp / max_bhp).clamp(0.0, 1.0) * pad.height();
            p.circle_stroke(Pos2::new(px, py), 4.0, Stroke::new(1.4_f32, ACCENT_GREEN));
        }
        if peak_nm > 1.0 {
            let tx = pad.left() + ((peak_nm_rpm - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0) * pad.width();
            let ty = pad.bottom() - (peak_nm / max_nm).clamp(0.0, 1.0) * pad.height();
            p.circle_stroke(Pos2::new(tx, ty), 3.6, Stroke::new(1.3_f32, ACCENT_AMBER));
        }
    }

    // Hover crosshair readout (when cursor is inside plot area) OR live RPM readout
    let hover_pos = ui.ctx().input(|i| i.pointer.hover_pos());
    let mut header_summary = format!(
        "Peak {:.0} BHP @ {:.1}k  •  {:.0} Nm @ {:.1}k",
        peak_bhp,
        peak_bhp_rpm * 0.001,
        peak_nm,
        peak_nm_rpm * 0.001
    );

    if let Some(hp) = hover_pos.filter(|pos| pad.contains(*pos)) {
        let nx = ((hp.x - pad.left()) / pad.width()).clamp(0.0, 1.0);
        let h_rpm = DYNO_MIN_RPM as f32 + nx * rpm_span;
        p.line_segment(
            [Pos2::new(hp.x, pad.top()), Pos2::new(hp.x, pad.bottom())],
            Stroke::new(0.9_f32, Color32::from_rgba_unmultiplied(228, 232, 240, 80)),
        );
        // Sample interpolated BHP & Nm at hover RPM
        if let Some((i_bhp, i_nm)) = sample_dyno_at_rpm(&spline_data, h_rpm) {
            let cy_bhp = pad.bottom() - (i_bhp / max_bhp).clamp(0.0, 1.0) * pad.height();
            let cy_nm = pad.bottom() - (i_nm / max_nm).clamp(0.0, 1.0) * pad.height();
            p.circle_filled(Pos2::new(hp.x, cy_bhp), 3.5, ACCENT_GREEN);
            p.circle_filled(Pos2::new(hp.x, cy_nm), 3.5, ACCENT_AMBER);
            header_summary = format!("{:.0} RPM  •  {:.0} BHP  •  {:.0} Nm", h_rpm, i_bhp, i_nm);
        }
    } else if snap.rpm >= DYNO_MIN_RPM as f64 {
        let nx = (((snap.rpm as f32) - DYNO_MIN_RPM as f32) / rpm_span).clamp(0.0, 1.0);
        let x = pad.left() + nx * pad.width();
        let y_bhp = pad.bottom() - ((snap.power_bhp as f32) / max_bhp).clamp(0.0, 1.0) * pad.height();
        let y_nm = pad.bottom() - ((snap.torque_nm as f32) / max_nm).clamp(0.0, 1.0) * pad.height();
        p.line_segment(
            [Pos2::new(x, pad.top()), Pos2::new(x, pad.bottom())],
            Stroke::new(0.8_f32, Color32::from_rgba_unmultiplied(228, 232, 240, 48)),
        );
        p.circle_filled(Pos2::new(x, y_nm), 2.8, ACCENT_AMBER);
        p.circle_filled(Pos2::new(x, y_bhp), 3.6, TEXT_PRIMARY);
    }

    p.text(
        Pos2::new(card_rect.left() + 48.0, card_rect.top() + 8.5),
        Align2::LEFT_TOP,
        header_summary,
        FontId::proportional(10.0),
        TEXT_PRIMARY,
    );
}

fn sample_dyno_at_rpm(samples: &[(f32, f32, f32)], target_rpm: f32) -> Option<(f32, f32)> {
    if samples.len() < 2 {
        return None;
    }
    if target_rpm < samples.first()?.0 || target_rpm > samples.last()?.0 {
        return None;
    }
    for w in samples.windows(2) {
        let (r0, p0, t0) = w[0];
        let (r1, p1, t1) = w[1];
        if target_rpm >= r0 && target_rpm <= r1 && (r1 - r0) > 1e-3 {
            let u = (target_rpm - r0) / (r1 - r0);
            return Some((p0 + (p1 - p0) * u, t0 + (t1 - t0) * u));
        }
    }
    None
}

fn draw_diag_card(ui: &mut egui::Ui, snap: &TelemetrySnapshot, diag_plot: &mut DiagnosticPlot) {
    let card_h = 176.0_f32;
    let card_w = ui.available_width().max(290.0);
    let (card_rect, _) = ui.allocate_exact_size(Vec2::new(card_w, card_h), Sense::hover());
    let p = ui.painter_at(card_rect);
    p.rect_filled(card_rect, 5.0, bg_card());
    p.rect_stroke(card_rect, 5.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let tab1 = Rect::from_min_size(Pos2::new(card_rect.left() + 8.0, card_rect.top() + 5.0), Vec2::new(66.0, 17.0));
    let tab2 = Rect::from_min_size(Pos2::new(card_rect.left() + 78.0, card_rect.top() + 5.0), Vec2::new(96.0, 17.0));
    if ui.interact(tab1, ui.id().with("tab_pv"), Sense::click()).clicked() {
        *diag_plot = DiagnosticPlot::PvDiagram;
    }
    if ui.interact(tab2, ui.id().with("tab_wave"), Sense::click()).clicked() {
        *diag_plot = DiagnosticPlot::Waveform;
    }
    p.text(
        tab1.center(),
        Align2::CENTER_CENTER,
        "P-V LOOP",
        FontId::proportional(10.0),
        if *diag_plot == DiagnosticPlot::PvDiagram { ACCENT_CYAN } else { TEXT_MUTED },
    );
    p.text(
        tab2.center(),
        Align2::CENTER_CENTER,
        "EXHAUST WAVE",
        FontId::proportional(10.0),
        if *diag_plot == DiagnosticPlot::Waveform { ACCENT_CYAN } else { TEXT_MUTED },
    );

    let pad = Rect::from_min_max(
        Pos2::new(card_rect.left() + 30.0, card_rect.top() + 28.0),
        Pos2::new(card_rect.right() - 12.0, card_rect.bottom() - 16.0),
    );

    match *diag_plot {
        DiagnosticPlot::PvDiagram => {
            p.text(
                Pos2::new(card_rect.right() - 10.0, card_rect.top() + 8.0),
                Align2::RIGHT_TOP,
                format!("{:.1} bar  |  {:.0} K", snap.cylinders[0].pressure_bar, snap.cylinders[0].temp_k),
                FontId::proportional(10.0),
                TEXT_PRIMARY,
            );
            for &(pr_tick, lbl) in &[(25.0, "25"), (50.0, "50"), (75.0, "75"), (100.0, "100")] {
                let y = pad.bottom() - (pr_tick / 105.0) * pad.height();
                p.line_segment(
                    [Pos2::new(pad.left(), y), Pos2::new(pad.right(), y)],
                    Stroke::new(0.6_f32, Color32::from_rgb(20, 24, 32)),
                );
                p.text(Pos2::new(pad.left() - 4.0, y), Align2::RIGHT_CENTER, lbl, FontId::proportional(9.0), TEXT_MUTED);
            }

            let v_min = (CLEARANCE_VOL_M3 * 1e6) as f32 * 0.9;
            let v_max = ((CLEARANCE_VOL_M3 + DISP_VOL_M3) * 1e6) as f32 * 1.03;
            let map_pt = |v: f32, pr: f32| -> Pos2 {
                let nx = ((v - v_min) / (v_max - v_min)).clamp(0.0, 1.0);
                let ny = (pr / 105.0).clamp(0.0, 1.0);
                Pos2::new(pad.left() + nx * pad.width(), pad.bottom() - ny * pad.height())
            };
            let mut pts = Vec::with_capacity(PV_POINTS + 1);
            let mut min_p = f32::MAX;
            let mut max_p = f32::MIN;
            for &(v, pr) in &snap.pv_trace {
                min_p = min_p.min(pr);
                max_p = max_p.max(pr);
                pts.push(map_pt(v, pr));
            }
            if pts.len() >= 3 && (max_p - min_p) > 0.8 {
                let fill_col = Color32::from_rgba_unmultiplied(92, 180, 230, 22);
                let center = pts.iter().fold(Vec2::ZERO, |acc, &pt| acc + pt.to_vec2()) / pts.len() as f32;
                let cp = center.to_pos2();
                let n = pts.len();
                for i in 0..n {
                    let a = pts[i];
                    let b = pts[(i + 1) % n];
                    let area2 = (a.x - cp.x) * (b.y - cp.y) - (b.x - cp.x) * (a.y - cp.y);
                    if area2.abs() > 1.0 {
                        let tri = if area2 > 0.0 { vec![cp, a, b] } else { vec![cp, b, a] };
                        p.add(egui::Shape::convex_polygon(tri, fill_col, Stroke::NONE));
                    }
                }
            }
            if let Some(&first) = pts.first() {
                pts.push(first);
            }
            p.add(egui::Shape::line(pts, Stroke::new(1.7_f32, ACCENT_CYAN)));
            let cur_v = ((CLEARANCE_VOL_M3 + snap.cylinders[0].piston_norm * DISP_VOL_M3) * 1e6) as f32;
            p.circle_filled(map_pt(cur_v, snap.cylinders[0].pressure_bar as f32), 3.4, ACCENT_AMBER);
        }
        DiagnosticPlot::Waveform => {
            let cy = pad.center().y;
            p.line_segment(
                [Pos2::new(pad.left(), cy), Pos2::new(pad.right(), cy)],
                Stroke::new(0.8_f32, Color32::from_rgb(24, 29, 38)),
            );
            let mut pts_l = Vec::with_capacity(SCOPE_LEN);
            let mut pts_r = Vec::with_capacity(SCOPE_LEN);
            let mut pts_a = Vec::with_capacity(SCOPE_LEN);
            let amp = pad.height() * 0.45;
            for i in 0..SCOPE_LEN {
                let x = pad.left() + (i as f32 / (SCOPE_LEN - 1) as f32) * pad.width();
                pts_l.push(Pos2::new(x, cy - snap.scope_left[i].clamp(-1.0, 1.0) * amp));
                pts_r.push(Pos2::new(x, cy - snap.scope_right[i].clamp(-1.0, 1.0) * amp));
                pts_a.push(Pos2::new(x, cy - snap.scope_audio[i].clamp(-1.0, 1.0) * amp));
            }
            p.add(egui::Shape::line(pts_l, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(92, 180, 230, 80))));
            p.add(egui::Shape::line(pts_r, Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(222, 158, 62, 80))));
            p.add(egui::Shape::line(pts_a, Stroke::new(1.6_f32, TEXT_PRIMARY)));
        }
    }
}

fn draw_vitals_strip(ui: &mut egui::Ui, snap: &TelemetrySnapshot, ctrl: &SharedControls) {
    let strip_w = ui.available_width().max(290.0);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(strip_w, 42.0), Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 5.0, bg_card());
    p.rect_stroke(rect, 5.0, Stroke::new(1.0_f32, BORDER_SUBTLE));

    let cells = [
        ("BLOCK", format!("{:.0}°C", snap.coolant_temp_c), snap.coolant_temp_c > 112.0),
        ("EGT", format!("{:.0}°C", snap.egt_c), snap.egt_c > 980.0),
        ("VALVE N2", format!("{:.0} b", ctrl.n2_pressure_bar), snap.valve_float_active),
        ("TRUMPET", format!("{:.0} mm", snap.trumpet_length_mm), false),
    ];
    let cw = rect.width() / 4.0;
    for (i, (lbl, val, warn)) in cells.iter().enumerate() {
        let cx = rect.left() + (i as f32 + 0.5) * cw;
        if i > 0 {
            let sx = rect.left() + i as f32 * cw;
            p.line_segment(
                [Pos2::new(sx, rect.top() + 7.0), Pos2::new(sx, rect.bottom() - 7.0)],
                Stroke::new(0.8_f32, BORDER_SUBTLE),
            );
        }
        p.text(Pos2::new(cx, rect.top() + 7.0), Align2::CENTER_TOP, *lbl, FontId::proportional(9.0), TEXT_MUTED);
        p.text(
            Pos2::new(cx, rect.top() + 20.0),
            Align2::CENTER_TOP,
            val,
            FontId::proportional(11.5),
            if *warn { ACCENT_RED } else { TEXT_PRIMARY },
        );
    }
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
