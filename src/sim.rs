use std::f64::consts::PI;

pub const NUM_CYL: usize = 10;
pub const PV_POINTS: usize = 144; // 5 deg resolution over 720 deg cycle
pub const DYNO_MIN_RPM: usize = 5000;
pub const DYNO_MAX_RPM: usize = 20000;
pub const DYNO_STEP_RPM: usize = 250;
pub const DYNO_BINS: usize = (DYNO_MAX_RPM - DYNO_MIN_RPM) / DYNO_STEP_RPM + 1;
pub const SCOPE_LEN: usize = 256;

/// Firing order: 1 - 6 - 5 - 10 - 2 - 7 - 3 - 8 - 4 - 9 (0-based indices)
pub const FIRING_ORDER: [usize; NUM_CYL] = [0, 5, 4, 9, 1, 6, 2, 7, 3, 8];

/// 2004-era 3.0L V10 Physical Dimensions
pub const BORE_M: f64 = 0.096;
pub const STROKE_M: f64 = 0.0414;
pub const CRANK_RADIUS_M: f64 = STROKE_M * 0.5;
pub const CONROD_M: f64 = 0.105;
pub const COMPRESSION_RATIO: f64 = 12.8;
pub const V_ANGLE_DEG: f64 = 72.0;

pub const PISTON_AREA_M2: f64 = PI * (BORE_M * 0.5) * (BORE_M * 0.5);
pub const DISP_VOL_M3: f64 = PISTON_AREA_M2 * STROKE_M;
pub const CLEARANCE_VOL_M3: f64 = DISP_VOL_M3 / (COMPRESSION_RATIO - 1.0);

const R_AIR: f64 = 287.05;
const GAMMA_AIR: f64 = 1.30; // Cycle-mean gamma for hot F1 combustion gases
const CV_GAS: f64 = R_AIR / (GAMMA_AIR - 1.0);
const P_ATM: f64 = 101_325.0;
const T_ATM: f64 = 298.15;
const Q_LHV_FUEL: f64 = 44.0e6; // J/kg gasoline
const AFR_STOICH: f64 = 14.7;
const ENGINE_INERTIA: f64 = 0.036; // kg*m^2 (ultra-light titanium crank + carbon clutch)
const FOUR_PI: f64 = 4.0 * PI;

// 7-speed F1 sequential gearbox (Monza 2004 ratios)
pub const GEAR_RATIOS: [f64; 7] = [2.85, 2.22, 1.80, 1.50, 1.29, 1.16, 1.08];
const FINAL_DRIVE: f64 = 5.85;
const WHEEL_RADIUS_M: f64 = 0.33;
const CAR_MASS_KG: f64 = 605.0;
const CAR_CDA: f64 = 1.12;

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BenchMode {
    FreeRev,
    DynoManualBrake,
    DynoRpmHold,
    DynoSweep,
    TrackManualGearbox,
    MonzaHotlapAuto,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StrokePhase {
    Power,
    Exhaust,
    Intake,
    Compression,
}

#[derive(Clone, Copy)]
pub struct SharedControls {
    // Test-cell rig & startup
    pub coolant_heater_on: bool,
    pub oil_heater_on: bool,
    pub n2_pressure_bar: f64,
    pub fuel_pump_on: bool,
    pub ignition_on: bool,
    pub starter_active: bool,

    // ECU & Induction tuning
    pub throttle: f64,
    pub idle_rpm_target: f64,
    pub rev_limit_rpm: f64,
    pub lambda_target: f64,
    pub spark_offset_deg: f64,
    pub trumpets_auto: bool,
    pub manual_trumpet_mm: f64,

    // Dyno & Gearbox
    pub bench_mode: BenchMode,
    pub dyno_brake_load: f64,
    pub dyno_hold_rpm: f64,
    pub requested_gear: usize,
    pub start_dyno_sweep: bool,
    pub clear_dyno_trace: bool,
    pub quick_start_trigger: bool,
    pub reset_repair_trigger: bool,
    pub manual_crank_delta_rad: f64,

    // Audio
    pub master_volume: f32,
    pub muted: bool,
}

impl SharedControls {
    pub fn preheat_on(&self) -> bool {
        self.coolant_heater_on || self.oil_heater_on
    }

    pub fn set_preheat(&mut self, on: bool) {
        self.coolant_heater_on = on;
        self.oil_heater_on = on;
    }

    pub fn toggle_pneumatics(&mut self) {
        self.n2_pressure_bar = if self.n2_pressure_bar >= 100.0 {
            0.0
        } else {
            165.0
        };
    }

    pub fn armed(&self) -> bool {
        self.fuel_pump_on && self.ignition_on
    }

    pub fn toggle_arm(&mut self) {
        let on = !self.armed();
        self.fuel_pump_on = on;
        self.ignition_on = on;
    }
}

impl Default for SharedControls {
    fn default() -> Self {
        Self {
            coolant_heater_on: false,
            oil_heater_on: false,
            n2_pressure_bar: 0.0,
            fuel_pump_on: false,
            ignition_on: false,
            starter_active: false,

            throttle: 0.0,
            idle_rpm_target: 4000.0,
            rev_limit_rpm: 19500.0,
            lambda_target: 0.88,
            spark_offset_deg: 0.0,
            trumpets_auto: true,
            manual_trumpet_mm: 105.0,

            bench_mode: BenchMode::FreeRev,
            dyno_brake_load: 0.35,
            dyno_hold_rpm: 14000.0,
            requested_gear: 1,
            start_dyno_sweep: false,
            clear_dyno_trace: false,
            quick_start_trigger: false,
            reset_repair_trigger: false,
            manual_crank_delta_rad: 0.0,

            master_volume: 0.65,
            muted: false,
        }
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct CylinderSnapshot {
    pub local_angle_rad: f64,
    pub piston_norm: f64,
    pub pressure_bar: f64,
    pub temp_k: f64,
    pub intake_lift_norm: f64,
    pub exhaust_lift_norm: f64,
    pub combustion_intensity: f64,
    pub spark_firing: bool,
    pub fuel_injecting: bool,
    pub phase: StrokePhase,
}

impl Default for CylinderSnapshot {
    fn default() -> Self {
        Self {
            local_angle_rad: 0.0,
            piston_norm: 0.0,
            pressure_bar: 1.013,
            temp_k: T_ATM,
            intake_lift_norm: 0.0,
            exhaust_lift_norm: 0.0,
            combustion_intensity: 0.0,
            spark_firing: false,
            fuel_injecting: false,
            phase: StrokePhase::Power,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct DynoPoint {
    pub rpm: f32,
    pub torque_nm: f32,
    pub power_bhp: f32,
    pub recorded: bool,
}

#[allow(dead_code)]
#[derive(Clone)]
pub struct TelemetrySnapshot {
    pub crank_angle_rad: f64,
    pub rpm: f64,
    pub torque_nm: f64,
    pub power_bhp: f64,
    pub plenum_pressure_bar: f64,
    pub ve_percent: f64,
    pub trumpet_length_mm: f64,
    pub coolant_temp_c: f64,
    pub oil_temp_c: f64,
    pub egt_c: f64,
    pub bore_health: f64,
    pub knock_count: u64,
    pub knock_active: bool,
    pub valve_float_active: bool,
    pub cold_seized: bool,
    pub rev_limiter_active: bool,
    pub is_running: bool,
    pub gear: usize,
    pub speed_kmh: f64,
    pub monza_dist_m: f64,
    pub monza_sector_name: &'static str,
    pub dyno_sweep_running: bool,
    pub effective_throttle: f64,
    pub required_n2_bar: f64,
    pub cylinders: [CylinderSnapshot; NUM_CYL],
    pub pv_trace: [(f32, f32); PV_POINTS],
    pub dyno_curve: [DynoPoint; DYNO_BINS],
    pub scope_left: [f32; SCOPE_LEN],
    pub scope_right: [f32; SCOPE_LEN],
    pub scope_audio: [f32; SCOPE_LEN],
}

impl Default for TelemetrySnapshot {
    fn default() -> Self {
        let mut dyno_curve = [DynoPoint::default(); DYNO_BINS];
        for (i, pt) in dyno_curve.iter_mut().enumerate() {
            pt.rpm = (DYNO_MIN_RPM + i * DYNO_STEP_RPM) as f32;
        }
        Self {
            crank_angle_rad: 0.0,
            rpm: 0.0,
            torque_nm: 0.0,
            power_bhp: 0.0,
            plenum_pressure_bar: 1.013,
            ve_percent: 0.0,
            trumpet_length_mm: 135.0,
            coolant_temp_c: 22.0,
            oil_temp_c: 22.0,
            egt_c: 25.0,
            bore_health: 1.0,
            knock_count: 0,
            knock_active: false,
            valve_float_active: false,
            cold_seized: false,
            rev_limiter_active: false,
            is_running: false,
            gear: 0,
            speed_kmh: 0.0,
            monza_dist_m: 0.0,
            monza_sector_name: "Pit Lane",
            dyno_sweep_running: false,
            effective_throttle: 0.0,
            required_n2_bar: 30.0,
            cylinders: [CylinderSnapshot::default(); NUM_CYL],
            pv_trace: [(0.0, 1.0); PV_POINTS],
            dyno_curve,
            scope_left: [0.0; SCOPE_LEN],
            scope_right: [0.0; SCOPE_LEN],
            scope_audio: [0.0; SCOPE_LEN],
        }
    }
}

/// Engine-Sim IR file (MIT-licensed from ange-yaghi/engine-sim)
const IR_WAV_BYTES: &[u8] = include_bytes!("../assets/minimal_muffling_01.wav");

/// Per-cylinder primary offsets (m) and acoustic attenuation from engine-sim V10 specs
const PRIMARY_OFFSETS_M: [f64; NUM_CYL] = [0.05, 0.04, 0.03, 0.02, 0.01, 0.05, 0.04, 0.03, 0.02, 0.01];
const SOUND_ATTENUATION: [f32; NUM_CYL] = [0.8, 1.0, 1.1, 0.9, 0.7, 0.7, 0.8, 1.0, 1.1, 0.7];
const PRIMARY_BASE_LEN_M: f64 = 0.45;
const BANK_COLLECTOR_LEN_M: [f64; 2] = [0.55, 0.60];

/// Compressible finite-volume gas node (ported from engine-sim's GasSystem)
#[derive(Clone, Copy)]
struct GasNode {
    n_mass: f64,
    e_internal: f64,
    volume: f64,
    area: f64,
    length: f64,
    momentum_x: f64,
}

impl GasNode {
    fn new(volume: f64, area: f64) -> Self {
        let length = (volume / area.max(1e-6)).max(0.05);
        let n_mass = (P_ATM * volume) / (R_AIR * T_ATM);
        let e_internal = n_mass * CV_GAS * T_ATM;
        Self {
            n_mass,
            e_internal,
            volume,
            area,
            length,
            momentum_x: 0.0,
        }
    }

    #[inline]
    fn temp_k(&self) -> f64 {
        (self.e_internal / (self.n_mass.max(1e-8) * CV_GAS)).clamp(200.0, 3400.0)
    }

    #[inline]
    fn pressure_pa(&self) -> f64 {
        (self.e_internal * (GAMMA_AIR - 1.0) / self.volume).clamp(8_000.0, 4.0e6)
    }

    #[inline]
    fn speed_of_sound(&self) -> f64 {
        (GAMMA_AIR * R_AIR * self.temp_k()).sqrt()
    }

    #[inline]
    fn dynamic_pressure(&self, dir_x: f64) -> f64 {
        if self.n_mass <= 1e-8 || self.e_internal <= 1e-8 {
            return 0.0;
        }
        let v = (self.momentum_x / self.n_mass) * dir_x;
        if v <= 0.0 {
            return 0.0;
        }
        let p_static = self.pressure_pa();
        let c = self.speed_of_sound();
        let mach2 = ((v * v) / (c * c)).min(1.0);
        let x = 1.0 + 0.5 * (GAMMA_AIR - 1.0) * mach2;
        p_static * (x.powf(GAMMA_AIR / (GAMMA_AIR - 1.0)) - 1.0)
    }

    #[inline]
    fn dissipate_excess_velocity(&mut self) {
        if self.n_mass <= 1e-8 {
            self.momentum_x = 0.0;
            return;
        }
        let v = self.momentum_x / self.n_mass;
        let c = self.speed_of_sound();
        if v.abs() > c {
            let v_new = c * v.signum();
            let de_k = 0.5 * self.n_mass * (v * v - v_new * v_new);
            self.momentum_x = self.n_mass * v_new;
            self.e_internal = (self.e_internal + de_k).max(self.n_mass * CV_GAS * 200.0);
        }
    }

    #[inline]
    fn update_velocity(&mut self, dt: f64, beta: f64) {
        if self.n_mass <= 1e-8 {
            return;
        }
        let p_fwd = self.dynamic_pressure(1.0);
        let p_bwd = self.dynamic_pressure(-1.0);
        let d_mom = (p_fwd - p_bwd) * self.area * dt * beta;
        let v0 = self.momentum_x / self.n_mass;
        self.momentum_x = (self.momentum_x - d_mom).clamp(-self.n_mass * 900.0, self.n_mass * 900.0);
        let v1 = self.momentum_x / self.n_mass;
        self.e_internal = (self.e_internal - 0.5 * self.n_mass * (v1 * v1 - v0 * v0))
            .max(self.n_mass * CV_GAS * 200.0);
        let _ = self.length;
    }

    #[inline]
    fn add_flow_from_cylinder(&mut self, dm: f64, cyl_temp: f64, dt: f64) {
        let m_old = self.n_mass.max(1e-8);
        let c = self.speed_of_sound();
        if dm > 0.0 {
            let dm_safe = dm.min(m_old * 0.45);
            let rho = (m_old / self.volume).max(0.05);
            let v_jet = ((dm_safe / dt.max(1e-7)) / (rho * self.area)).clamp(0.0, c);
            self.n_mass += dm_safe;
            self.e_internal += dm_safe * GAMMA_AIR * CV_GAS * cyl_temp;
            self.momentum_x += v_jet * dm_safe;
        } else if dm < 0.0 {
            let out = (-dm).min(m_old * 0.35);
            let frac = out / m_old;
            self.n_mass -= out;
            self.e_internal *= 1.0 - frac;
            self.momentum_x *= 1.0 - frac;
        }
    }

    #[inline]
    fn flow_to_node(&mut self, sink: &mut GasNode, dt: f64, flow_area: f64) {
        let p0 = self.pressure_pa() + self.dynamic_pressure(1.0);
        let p1 = sink.pressure_pa() + sink.dynamic_pressure(-1.0);
        let dm_rate = orifice_mass_flow(p0, self.temp_k(), p1, sink.temp_k(), flow_area);
        let mut dm = dm_rate * dt;
        let max_src = self.n_mass * 0.25;
        let max_snk = sink.n_mass * 0.25;
        dm = dm.clamp(-max_snk, max_src);
        if dm > 0.0 {
            let frac = dm / self.n_mass.max(1e-8);
            let de = self.e_internal * GAMMA_AIR * frac;
            let dp = self.momentum_x * frac;
            let rho_snk = (sink.n_mass / sink.volume).max(0.05);
            let v_jet = ((dm / dt.max(1e-7)) / (rho_snk * sink.area)).clamp(0.0, sink.speed_of_sound());
            self.n_mass -= dm;
            self.e_internal = (self.e_internal - de).max(self.n_mass * CV_GAS * 200.0);
            self.momentum_x -= dp;
            sink.n_mass += dm;
            sink.e_internal += de;
            sink.momentum_x += dp + v_jet * dm;
        } else if dm < 0.0 {
            let back = -dm;
            let frac = back / sink.n_mass.max(1e-8);
            let de = sink.e_internal * GAMMA_AIR * frac;
            sink.n_mass -= back;
            sink.e_internal = (sink.e_internal - de).max(sink.n_mass * CV_GAS * 200.0);
            sink.momentum_x *= 1.0 - frac;
            self.n_mass += back;
            self.e_internal += de;
        }
    }

    #[inline]
    fn flow_to_atmosphere(&mut self, dt: f64, flow_area: f64) {
        let p0 = self.pressure_pa() + self.dynamic_pressure(1.0);
        let dm_rate = orifice_mass_flow(p0, self.temp_k(), P_ATM, T_ATM, flow_area);
        let dm = (dm_rate * dt).clamp(-self.n_mass * 0.2, self.n_mass * 0.35);
        if dm > 0.0 {
            let frac = dm / self.n_mass.max(1e-8);
            self.n_mass -= dm;
            self.e_internal = (self.e_internal * (1.0 - GAMMA_AIR * frac))
                .max(self.n_mass * CV_GAS * 200.0);
            self.momentum_x *= 1.0 - frac;
        } else if dm < 0.0 {
            let inflow = -dm;
            self.n_mass += inflow;
            self.e_internal += inflow * GAMMA_AIR * CV_GAS * T_ATM;
        }
    }
}

#[derive(Clone, Copy)]
struct Cylinder {
    firing_offset_rad: f64,
    air_mass: f64,
    fresh_air_mass: f64,
    fuel_mass: f64,
    unburnt_fuel: f64,
    temp_k: f64,
    pressure_pa: f64,
    comb_progress: f64,
    combusting: bool,
    comb_start_rad: f64,
    comb_energy_total: f64,
    prev_local_angle: f64,
    spark_timer: f64,
    knock_timer: f64,
    last_intake_mass: f64,
    intake_accum_mass: f64,
    intake_lift: f64,
    exhaust_lift: f64,
    piston_x: f64,
    spark_cut_this_cycle: bool,
    runner_gas: GasNode,
}

impl Cylinder {
    fn new(idx: usize, firing_offset_rad: f64) -> Self {
        let vol = CLEARANCE_VOL_M3;
        let air_mass = (P_ATM * vol) / (R_AIR * T_ATM);
        let runner_area = 0.00185;
        let runner_vol = 50.0e-6 + runner_area * (PRIMARY_BASE_LEN_M + PRIMARY_OFFSETS_M[idx]);
        Self {
            firing_offset_rad,
            air_mass,
            fresh_air_mass: air_mass,
            fuel_mass: 0.0,
            unburnt_fuel: 0.0,
            temp_k: T_ATM,
            pressure_pa: P_ATM,
            comb_progress: 0.0,
            combusting: false,
            comb_start_rad: 0.0,
            comb_energy_total: 0.0,
            prev_local_angle: 0.0,
            spark_timer: 0.0,
            knock_timer: 0.0,
            last_intake_mass: 0.0,
            intake_accum_mass: 0.0,
            intake_lift: 0.0,
            exhaust_lift: 0.0,
            piston_x: 0.0,
            spark_cut_this_cycle: false,
            runner_gas: GasNode::new(runner_vol, runner_area),
        }
    }
}

/// Fixed-sample delay line (ported from engine-sim's DelayFilter)
struct DelayFilter {
    buf: Vec<f32>,
    ptr: usize,
    delay: usize,
}

impl DelayFilter {
    fn new(delay_sec: f64, sample_rate: f64) -> Self {
        let delay = (delay_sec * sample_rate).round().max(1.0) as usize;
        Self {
            buf: vec![0.0; delay + 4],
            ptr: 0,
            delay,
        }
    }

    #[inline]
    fn fast_f(&mut self, sample: f32) -> f32 {
        let len = self.buf.len();
        let read_idx = (self.ptr + len - self.delay) % len;
        let out = self.buf[read_idx];
        self.buf[self.ptr] = sample;
        self.ptr = (self.ptr + 1) % len;
        out
    }
}

/// 4th-order Butterworth low-pass filter (ported from engine-sim's ButterworthLowPassFilter)
#[derive(Clone, Copy)]
struct ButterworthLP {
    a: [f64; 5],
    f4: f64,
    x: [f64; 4],
    y: [f64; 4],
}

impl ButterworthLP {
    fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        let fc = cutoff_hz.clamp(sample_rate * 0.001, sample_rate * 0.46);
        let f = (PI * fc / sample_rate).tan();
        let f2 = f * f;
        let f3 = f2 * f;
        let f4 = f2 * f2;
        let m = -2.0 * (5.0 * PI / 8.0).cos();
        let n = -2.0 * (7.0 * PI / 8.0).cos();

        let a0 = 1.0 + (m + n) * f + (2.0 + n * m) * f2 + (m + n) * f3 + f4;
        let a = [
            a0,
            (-4.0 - 2.0 * (n + m) * f + 2.0 * (m + n) * f3 + 4.0 * f4) / a0,
            (6.0 - 2.0 * (2.0 + m * n) * f2 + 6.0 * f4) / a0,
            (-4.0 + 2.0 * (m + n) * f - 2.0 * (m + n) * f3 + 4.0 * f4) / a0,
            (1.0 - (n + m) * f + (2.0 + m * n) * f2 - (m + n) * f3 + f4) / a0,
        ];
        Self {
            a,
            f4,
            x: [0.0; 4],
            y: [0.0; 4],
        }
    }

    #[inline]
    fn fast_f(&mut self, sample: f32) -> f32 {
        let s = sample as f64;
        let n = (self.f4 / self.a[0])
            * (s + 4.0 * self.x[0] + 6.0 * self.x[1] + 4.0 * self.x[2] + self.x[3]);
        let d = -self.a[1] * self.y[0]
            - self.a[2] * self.y[1]
            - self.a[3] * self.y[2]
            - self.a[4] * self.y[3];
        let y = n + d;

        self.x[3] = self.x[2];
        self.x[2] = self.x[1];
        self.x[1] = self.x[0];
        self.x[0] = s;

        self.y[3] = self.y[2];
        self.y[2] = self.y[1];
        self.y[1] = self.y[0];
        self.y[0] = y;

        y as f32
    }
}

/// 1st-order DC blocking low-pass filter (ported from engine-sim's LowPassFilter)
#[derive(Clone, Copy)]
struct LowPassFilter {
    alpha: f32,
    y: f32,
}

impl LowPassFilter {
    fn new(cutoff_hz: f64, sample_rate: f64) -> Self {
        let dt = 1.0 / sample_rate;
        let rc = 1.0 / (cutoff_hz * 2.0 * PI);
        let alpha = (dt / (rc + dt)) as f32;
        Self { alpha, y: 0.0 }
    }

    #[inline]
    fn fast_f(&mut self, sample: f32) -> f32 {
        self.y += self.alpha * (sample - self.y);
        self.y
    }
}

/// Phase jitter filter (ported from engine-sim's JitterFilter)
struct JitterFilter {
    history: [f32; 10],
    offset: usize,
    jitter_scale: f32,
    noise_lp: ButterworthLP,
}

impl JitterFilter {
    fn new(cutoff_hz: f64, sample_rate: f64, jitter_scale: f32) -> Self {
        Self {
            history: [0.0; 10],
            offset: 0,
            jitter_scale,
            noise_lp: ButterworthLP::new(cutoff_hz, sample_rate),
        }
    }

    #[inline]
    fn fast_f(&mut self, sample: f32, uniform_0_1: f32) -> f32 {
        self.history[self.offset] = sample;
        self.offset = (self.offset + 1) % 10;

        let s = self
            .noise_lp
            .fast_f(uniform_0_1 * 9.0 * self.jitter_scale)
            .clamp(0.0, 9.0);
        let s0 = s.floor() as usize;
        let s1 = s.ceil() as usize;
        let frac = s - s0 as f32;

        let i0 = (self.offset + s0) % 10;
        let i1 = (self.offset + s1) % 10;
        self.history[i0] * (1.0 - frac) + self.history[i1] * frac
    }
}

/// Real-time FIR impulse response convolver (ported from engine-sim's ConvolutionFilter)
struct ConvolutionFilter {
    kernel: Vec<f32>,
    history: Vec<f32>,
    ptr: usize,
    len: usize,
}

impl ConvolutionFilter {
    fn from_wav_bytes(bytes: &[u8], target_sr: f64, volume: f32, max_taps: usize) -> Self {
        let mut wav_sr = 44100.0_f64;
        let mut raw_samples = Vec::<f32>::new();

        if bytes.len() > 12 && &bytes[0..4] == b"RIFF" && &bytes[8..12] == b"WAVE" {
            let mut pos = 12usize;
            while pos + 8 <= bytes.len() {
                let id = &bytes[pos..pos + 4];
                let size = u32::from_le_bytes([
                    bytes[pos + 4],
                    bytes[pos + 5],
                    bytes[pos + 6],
                    bytes[pos + 7],
                ]) as usize;
                let data_start = pos + 8;
                let data_end = (data_start + size).min(bytes.len());
                if id == b"fmt " && data_end - data_start >= 16 {
                    wav_sr = u32::from_le_bytes([
                        bytes[data_start + 4],
                        bytes[data_start + 5],
                        bytes[data_start + 6],
                        bytes[data_start + 7],
                    ]) as f64;
                } else if id == b"data" {
                    for chunk in bytes[data_start..data_end].chunks_exact(2) {
                        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
                        raw_samples.push(s as f32 / i16::MAX as f32);
                    }
                }
                pos = data_start + size + (size & 1);
            }
        }

        if raw_samples.is_empty() {
            raw_samples.push(1.0);
        }

        // Linearly resample IR from WAV rate (44.1 kHz) to host WASAPI rate (e.g. 48 kHz)
        let ratio = wav_sr / target_sr.max(8000.0);
        let resampled_len = ((raw_samples.len() as f64 / ratio).floor() as usize).clamp(1, max_taps);
        let mut kernel = Vec::with_capacity(resampled_len);
        for i in 0..resampled_len {
            let src_pos = i as f64 * ratio;
            let idx = (src_pos.floor() as usize).min(raw_samples.len() - 1);
            let next = (idx + 1).min(raw_samples.len() - 1);
            let frac = (src_pos - idx as f64) as f32;
            let val = raw_samples[idx] * (1.0 - frac) + raw_samples[next] * frac;
            kernel.push(val * volume);
        }

        let len = kernel.len();
        Self {
            kernel,
            history: vec![0.0; len * 2],
            ptr: 0,
            len,
        }
    }

    #[inline]
    fn fast_f(&mut self, sample: f32) -> f32 {
        self.ptr = if self.ptr == 0 { self.len - 1 } else { self.ptr - 1 };
        self.history[self.ptr] = sample;
        self.history[self.ptr + self.len] = sample;
        let h = &self.history[self.ptr..self.ptr + self.len];
        let mut acc = 0.0_f32;
        for (k, s) in self.kernel.iter().zip(h.iter()) {
            acc += k * s;
        }
        acc
    }
}

/// Auto-gain peak limiter (ported from engine-sim's LevelingFilter)
struct LevelingFilter {
    peak: f32,
    attenuation: f32,
    target: f32,
    max_level: f32,
    min_level: f32,
}

impl LevelingFilter {
    fn new(target: f32, max_level: f32, min_level: f32) -> Self {
        Self {
            peak: target,
            attenuation: 1.0,
            target,
            max_level,
            min_level,
        }
    }

    #[inline]
    fn update_gain(&mut self, mag: f32) -> f32 {
        self.peak *= 0.999;
        if mag > self.peak {
            self.peak = mag;
        }
        if self.peak <= 1e-9 {
            return self.attenuation;
        }
        let raw = (self.target / self.peak).clamp(self.min_level, self.max_level);
        self.attenuation = 0.9 * self.attenuation + 0.1 * raw;
        self.attenuation
    }
}

struct SynthChannel {
    antialiasing: ButterworthLP,
    jitter: JitterFilter,
    dc_filter: LowPassFilter,
    air_noise_lp: ButterworthLP,
    convolution: ConvolutionFilter,
    prev_in: f32,
}

impl SynthChannel {
    fn new(sample_rate: f64) -> Self {
        Self {
            antialiasing: ButterworthLP::new(2600.0, sample_rate),
            jitter: JitterFilter::new(10000.0, sample_rate, 0.15),
            dc_filter: LowPassFilter::new(10.0, sample_rate),
            air_noise_lp: ButterworthLP::new(2000.0, sample_rate),
            convolution: ConvolutionFilter::from_wav_bytes(IR_WAV_BYTES, sample_rate, 0.01, 2048),
            prev_in: 0.0,
        }
    }

    #[inline]
    fn process(&mut self, raw_signal: f32, sample_rate: f32, u01: f32, white_n: f32) -> f32 {
        let aa = self.antialiasing.fast_f(raw_signal);
        let f_in = self.jitter.fast_f(aa, u01);
        let f_dc = self.dc_filter.fast_f(f_in);
        let f = f_in - f_dc;
        let f_p = (f_in - self.prev_in) * sample_rate;
        self.prev_in = f_in;

        let r = self.air_noise_lp.fast_f(white_n);
        let df_f_mix = 0.012_f32;
        let mut v_in = f_p * df_f_mix + f * r * (1.0 - df_f_mix);
        if !v_in.is_finite() || v_in.is_subnormal() {
            v_in = 0.0;
        }
        self.convolution.fast_f(v_in)
    }
}

pub struct EngineSim {
    sample_rate: f64,
    cylinders: [Cylinder; NUM_CYL],
    crank_angle_rad: f64,
    omega_rad_s: f64,

    // Thermodynamic & fluid state
    plenum_pressure_pa: f64,
    trumpet_length_m: f64,
    coolant_temp_c: f64,
    oil_temp_c: f64,
    egt_c: f64,
    bore_health: f64,
    knock_count: u64,
    knock_active_timer: f64,
    valve_float_active: bool,
    cold_seized: bool,

    // Control / ECU internal state
    idle_integral: f64,
    effective_throttle: f64,
    rev_cut_timer: f64,
    post_cut_timer: f64,
    rev_limiter_active: bool,
    shift_cut_timer: f64,
    shift_sync_timer: f64,
    blip_timer: f64,
    shift_cooldown: f64,
    last_gear: usize,

    // Cycle-averaged torque & filtered telemetry
    firing_work_buf: [f64; NUM_CYL],
    firing_angle_buf: [f64; NUM_CYL],
    firing_slot: usize,
    interval_work_accum: f64,
    interval_angle_accum: f64,
    cycle_mean_torque_nm: f64,
    filtered_torque_nm: f64,
    filtered_power_bhp: f64,
    filtered_ve: f64,
    pv_trace: [(f32, f32); PV_POINTS],

    // Dyno sweep state
    dyno_sweep_running: bool,
    dyno_sweep_rpm: f64,
    dyno_curve: [DynoPoint; DYNO_BINS],

    // Car / Monza state
    car_speed_ms: f64,
    monza_dist_m: f64,
    monza_sector_name: &'static str,

    // Audio DSP state (engine-sim architecture)
    collectors: [GasNode; 2],
    delay_filters: [DelayFilter; NUM_CYL],
    synth_ch: [SynthChannel; 2],
    out_antialias_l: ButterworthLP,
    out_antialias_r: ButterworthLP,
    leveler: LevelingFilter,
    rng_state: u32,
    starter_phase: f64,

    // Oscilloscope ring buffers
    scope_left: [f32; SCOPE_LEN],
    scope_right: [f32; SCOPE_LEN],
    scope_audio: [f32; SCOPE_LEN],
    scope_ptr: usize,
    scope_decim: usize,
}

impl EngineSim {
    pub fn new(sample_rate: f64) -> Self {
        let mut firing_offsets = [0.0; NUM_CYL];
        for (slot, &cyl_idx) in FIRING_ORDER.iter().enumerate() {
            firing_offsets[cyl_idx] = (slot as f64) * (72.0_f64.to_radians());
        }

        let cylinders = std::array::from_fn(|i| Cylinder::new(i, firing_offsets[i]));
        let mut dyno_curve = [DynoPoint::default(); DYNO_BINS];
        for (i, pt) in dyno_curve.iter_mut().enumerate() {
            pt.rpm = (DYNO_MIN_RPM + i * DYNO_STEP_RPM) as f32;
        }

        let sr_safe = sample_rate.max(22050.0);
        let delay_filters = std::array::from_fn(|i| {
            let bank = i / 5;
            let ex_len = PRIMARY_BASE_LEN_M + PRIMARY_OFFSETS_M[i] + BANK_COLLECTOR_LEN_M[bank];
            DelayFilter::new(ex_len / 343.0, sr_safe)
        });
        let col_area = 0.0052;
        let collectors = [
            GasNode::new(col_area * BANK_COLLECTOR_LEN_M[0], col_area),
            GasNode::new(col_area * BANK_COLLECTOR_LEN_M[1], col_area),
        ];
        let mut sim = Self {
            sample_rate: sr_safe,
            cylinders,
            crank_angle_rad: 0.0,
            omega_rad_s: 0.0,

            plenum_pressure_pa: P_ATM,
            trumpet_length_m: 0.135,
            coolant_temp_c: 22.0,
            oil_temp_c: 22.0,
            egt_c: 25.0,
            bore_health: 1.0,
            knock_count: 0,
            knock_active_timer: 0.0,
            valve_float_active: false,
            cold_seized: false,

            idle_integral: 0.05,
            effective_throttle: 0.0,
            rev_cut_timer: 0.0,
            post_cut_timer: 0.0,
            rev_limiter_active: false,
            shift_cut_timer: 0.0,
            shift_sync_timer: 0.0,
            blip_timer: 0.0,
            shift_cooldown: 0.0,
            last_gear: 1,

            firing_work_buf: [0.0; NUM_CYL],
            firing_angle_buf: [0.0; NUM_CYL],
            firing_slot: 0,
            interval_work_accum: 0.0,
            interval_angle_accum: 0.0,
            cycle_mean_torque_nm: 0.0,
            filtered_torque_nm: 0.0,
            filtered_power_bhp: 0.0,
            filtered_ve: 0.0,
            pv_trace: [(30.0, 1.0); PV_POINTS],

            dyno_sweep_running: false,
            dyno_sweep_rpm: 5500.0,
            dyno_curve,

            car_speed_ms: 0.0,
            monza_dist_m: 0.0,
            monza_sector_name: "Pit Lane",

            collectors,
            delay_filters,
            synth_ch: [SynthChannel::new(sr_safe), SynthChannel::new(sr_safe)],
            out_antialias_l: ButterworthLP::new(sr_safe * 0.45, sr_safe),
            out_antialias_r: ButterworthLP::new(sr_safe * 0.45, sr_safe),
            leveler: LevelingFilter::new(26000.0, 1.9, 1.0e-6),
            rng_state: 0x1357_2468,
            starter_phase: 0.0,

            scope_left: [0.0; SCOPE_LEN],
            scope_right: [0.0; SCOPE_LEN],
            scope_audio: [0.0; SCOPE_LEN],
            scope_ptr: 0,
            scope_decim: 0,
        };
        sim.sync_cylinders_kinematics();
        sim
    }

    pub fn quick_start(&mut self, ctrl: &mut SharedControls) {
        ctrl.coolant_heater_on = true;
        ctrl.oil_heater_on = true;
        ctrl.n2_pressure_bar = 165.0;
        ctrl.fuel_pump_on = true;
        ctrl.ignition_on = true;
        ctrl.starter_active = false;
        ctrl.quick_start_trigger = false;

        self.coolant_temp_c = 82.0;
        self.oil_temp_c = 85.0;
        self.egt_c = 460.0;
        self.cold_seized = false;
        self.omega_rad_s = (ctrl.idle_rpm_target * 2.0 * PI / 60.0).max(400.0);
        self.idle_integral = 0.052;
        self.plenum_pressure_pa = 32_000.0;
    }

    pub fn repair_and_reset(&mut self, ctrl: &mut SharedControls) {
        ctrl.reset_repair_trigger = false;
        ctrl.starter_active = false;
        ctrl.ignition_on = false;
        ctrl.fuel_pump_on = false;
        ctrl.n2_pressure_bar = 0.0;
        ctrl.coolant_heater_on = false;
        ctrl.oil_heater_on = false;
        ctrl.throttle = 0.0;
        ctrl.bench_mode = BenchMode::FreeRev;

        self.omega_rad_s = 0.0;
        self.coolant_temp_c = 22.0;
        self.oil_temp_c = 22.0;
        self.egt_c = 25.0;
        self.bore_health = 1.0;
        self.knock_count = 0;
        self.knock_active_timer = 0.0;
        self.valve_float_active = false;
        self.cold_seized = false;
        self.car_speed_ms = 0.0;
        self.monza_dist_m = 0.0;
        self.dyno_sweep_running = false;
        self.plenum_pressure_pa = P_ATM;
        self.post_cut_timer = 0.0;
        self.shift_sync_timer = 0.0;
        self.firing_work_buf = [0.0; NUM_CYL];
        self.firing_angle_buf = [0.0; NUM_CYL];
        self.interval_work_accum = 0.0;
        self.interval_angle_accum = 0.0;
        self.cycle_mean_torque_nm = 0.0;
        self.filtered_torque_nm = 0.0;
        self.filtered_power_bhp = 0.0;

        let col_area = 0.0052;
        self.collectors = [
            GasNode::new(col_area * BANK_COLLECTOR_LEN_M[0], col_area),
            GasNode::new(col_area * BANK_COLLECTOR_LEN_M[1], col_area),
        ];

        for (i, c) in self.cylinders.iter_mut().enumerate() {
            *c = Cylinder::new(i, c.firing_offset_rad);
        }
        self.sync_cylinders_kinematics();
    }

    fn sync_cylinders_kinematics(&mut self) {
        for c in &mut self.cylinders {
            let theta = wrap_720(self.crank_angle_rad - c.firing_offset_rad);
            let (x, _, _) = slider_crank(theta);
            c.piston_x = x;
            c.prev_local_angle = theta;
        }
    }

    #[inline]
    fn fast_noise(&mut self) -> f32 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.rng_state = x;
        ((x as i32) as f32) * (1.0 / 2_147_483_648.0)
    }

    /// Advance 1 audio sample (2 physics substeps) and return stereo audio (L, R)
    pub fn step_sample(&mut self, ctrl: &mut SharedControls) -> (f32, f32) {
        if ctrl.quick_start_trigger {
            self.quick_start(ctrl);
        }
        if ctrl.reset_repair_trigger {
            self.repair_and_reset(ctrl);
        }
        if ctrl.clear_dyno_trace {
            ctrl.clear_dyno_trace = false;
            for pt in &mut self.dyno_curve {
                pt.recorded = false;
                pt.torque_nm = 0.0;
                pt.power_bhp = 0.0;
            }
        }
        if ctrl.start_dyno_sweep {
            ctrl.start_dyno_sweep = false;
            if self.coolant_temp_c < 60.0 || self.oil_temp_c < 60.0 || !ctrl.armed() || ctrl.n2_pressure_bar < 100.0 {
                self.quick_start(ctrl);
            }
            ctrl.bench_mode = BenchMode::DynoSweep;
            self.dyno_sweep_running = true;
            self.dyno_sweep_rpm = 5500.0;
            if self.omega_rad_s < 520.0 {
                self.omega_rad_s = 5500.0 * 2.0 * PI / 60.0;
            }
        }
        if ctrl.manual_crank_delta_rad.abs() > 1e-7 {
            let delta = ctrl.manual_crank_delta_rad;
            ctrl.manual_crank_delta_rad = 0.0;
            if self.omega_rad_s < 15.0 {
                let min_temp = self.coolant_temp_c.min(self.oil_temp_c);
                let resistance = if min_temp < 58.0 { 0.08 } else { 1.0 };
                let d_theta = delta * resistance;
                self.crank_angle_rad = wrap_720(self.crank_angle_rad + d_theta);
                for idx in 0..NUM_CYL {
                    let cyl = &mut self.cylinders[idx];
                    let theta = wrap_720(self.crank_angle_rad - cyl.firing_offset_rad);
                    cyl.prev_local_angle = theta;
                    let (x, _, _) = slider_crank(theta);
                    cyl.piston_x = x;
                    let vol = CLEARANCE_VOL_M3 + PISTON_AREA_M2 * x;
                    cyl.intake_lift = intake_lift_norm(theta);
                    cyl.exhaust_lift = exhaust_lift_norm(theta);
                    if cyl.intake_lift > 0.05 || cyl.exhaust_lift > 0.05 {
                        cyl.pressure_pa += (P_ATM - cyl.pressure_pa) * 0.35;
                        cyl.air_mass = (cyl.pressure_pa * vol) / (R_AIR * cyl.temp_k);
                    } else {
                        cyl.pressure_pa = (cyl.air_mass * R_AIR * cyl.temp_k / vol).clamp(20_000.0, 4.0e6);
                    }
                    if idx == 0 {
                        let pv_idx = (((theta / FOUR_PI) * PV_POINTS as f64) as usize).min(PV_POINTS - 1);
                        self.pv_trace[pv_idx] = ((vol * 1e6) as f32, (cyl.pressure_pa * 1e-5) as f32);
                    }
                }
            }
        }

        const SUBSTEPS: usize = 2;
        let dt = 1.0 / (self.sample_rate * SUBSTEPS as f64);

        for _ in 0..SUBSTEPS {
            self.step_physics_substep(dt, ctrl);
        }

        // Write gas-dynamic exhaust pulses into the 2 synthesizer channels (engine-sim writeToSynthesizer)
        let attenuation = (self.omega_rad_s.abs() / 40.0).clamp(0.0, 1.0);
        let attenuation_3 = attenuation * attenuation * attenuation;

        let mut staging = [0.0_f32; 2];
        for i in 0..NUM_CYL {
            let bank = i / 5;
            let ex_len = PRIMARY_BASE_LEN_M + PRIMARY_OFFSETS_M[i] + BANK_COLLECTOR_LEN_M[bank];
            let runner = &self.cylinders[i].runner_gas;
            let exhaust_flow = attenuation_3
                * 1600.0
                * ((runner.pressure_pa() - P_ATM)
                    + 0.1 * runner.dynamic_pressure(1.0)
                    + 0.1 * runner.dynamic_pressure(-1.0));
            let delayed = self.delay_filters[i].fast_f(exhaust_flow as f32);
            staging[bank] += SOUND_ATTENUATION[i]
                * (0.008 * delayed / NUM_CYL as f32)
                / (ex_len * ex_len) as f32;
        }

        let sr_f32 = self.sample_rate as f32;
        let u0 = 0.5 + 0.5 * self.fast_noise();
        let w0 = self.fast_noise();
        let u1 = 0.5 + 0.5 * self.fast_noise();
        let w1 = self.fast_noise();

        let v0 = self.synth_ch[0].process(staging[0], sr_f32, u0, w0);
        let v1 = self.synth_ch[1].process(staging[1], sr_f32, u1, w1);

        let aa_l = self.out_antialias_l.fast_f(v0 * 0.72 + v1 * 0.28);
        let aa_r = self.out_antialias_r.fast_f(v1 * 0.72 + v0 * 0.28);

        let level_gain = self.leveler.update_gain(aa_l.abs().max(aa_r.abs()));
        let norm_scale = (level_gain / 32768.0) * if ctrl.muted { 0.0 } else { ctrl.master_volume };

        // External pneumatic starter gun turbine sound
        let rpm = (self.omega_rad_s * 60.0 / (2.0 * PI)).max(0.0);
        let starter_snd = if ctrl.starter_active {
            let st_freq = 440.0 + rpm * 0.35;
            self.starter_phase = (self.starter_phase + 2.0 * PI * st_freq / self.sample_rate) % (2.0 * PI);
            let air_hiss = self.fast_noise() * 0.045;
            (self.starter_phase.sin() as f32 * 0.09) + air_hiss
        } else {
            0.0
        };

        let out_l = (aa_l * norm_scale + starter_snd).tanh();
        let out_r = (aa_r * norm_scale + starter_snd).tanh();

        // Update oscilloscope trace (decimated 2:1)
        self.scope_decim = (self.scope_decim + 1) & 1;
        if self.scope_decim == 0 {
            let sp = self.scope_ptr;
            let disp_scale = level_gain / 32768.0;
            self.scope_left[sp] = (v0 * disp_scale).clamp(-1.0, 1.0);
            self.scope_right[sp] = (v1 * disp_scale).clamp(-1.0, 1.0);
            self.scope_audio[sp] = 0.5 * (out_l + out_r);
            self.scope_ptr = (sp + 1) % SCOPE_LEN;
        }

        (out_l, out_r)
    }

    fn step_physics_substep(&mut self, dt: f64, ctrl: &mut SharedControls) {
        let mut rpm = self.omega_rad_s * 60.0 / (2.0 * PI);

        // 1. Monza automated qualifying lap or Gearbox shift management
        let mut requested_throttle = ctrl.throttle.clamp(0.0, 1.0);
        let mut brake_torque_car = 0.0;

        if self.shift_cooldown > 0.0 {
            self.shift_cooldown = (self.shift_cooldown - dt).max(0.0);
        }

        if ctrl.bench_mode == BenchMode::MonzaHotlapAuto {
            let (auto_tps, auto_brake, target_g, sector) =
                monza_driver(self.monza_dist_m, self.car_speed_ms, self.last_gear);
            requested_throttle = auto_tps;
            brake_torque_car = auto_brake;
            if self.shift_cooldown <= 0.0 {
                ctrl.requested_gear = target_g;
            } else {
                ctrl.requested_gear = self.last_gear;
            }
            self.monza_sector_name = sector;
        } else if ctrl.bench_mode == BenchMode::TrackManualGearbox {
            self.monza_sector_name = "Test Track";
        } else {
            self.monza_sector_name = "Dyno Cell";
        }

        // Handle gear changes with smooth dog-box synchronization (no instantaneous RPM or acoustic discontinuity)
        let in_gearbox_mode = matches!(
            ctrl.bench_mode,
            BenchMode::TrackManualGearbox | BenchMode::MonzaHotlapAuto
        );
        if in_gearbox_mode {
            let req_g = ctrl.requested_gear.clamp(1, 7);
            if req_g > self.last_gear {
                self.shift_cut_timer = 0.030;
                self.shift_sync_timer = 0.036;
                self.shift_cooldown = 0.10;
                self.last_gear = req_g;
            } else if req_g < self.last_gear {
                self.blip_timer = 0.042;
                self.shift_sync_timer = 0.036;
                self.shift_cooldown = 0.10;
                self.last_gear = req_g;
            }
        } else {
            self.last_gear = ctrl.requested_gear.clamp(1, 7);
        }

        if self.shift_cut_timer > 0.0 {
            self.shift_cut_timer = (self.shift_cut_timer - dt).max(0.0);
        }
        if self.blip_timer > 0.0 {
            self.blip_timer = (self.blip_timer - dt).max(0.0);
            requested_throttle = requested_throttle.max(0.75);
        }

        // Dyno sweep override (finish 250 RPM below rev limiter so limiter cut never pollutes peak bin)
        if ctrl.bench_mode == BenchMode::DynoSweep && self.dyno_sweep_running {
            requested_throttle = 1.0;
            self.dyno_sweep_rpm += 2200.0 * dt;
            let sweep_end = (ctrl.rev_limit_rpm - 250.0).clamp(10000.0, 20000.0);
            if self.dyno_sweep_rpm >= sweep_end {
                self.dyno_sweep_running = false;
                ctrl.bench_mode = BenchMode::DynoManualBrake;
                ctrl.dyno_brake_load = 0.25;
            }
        }

        // 2. Idle Air Control + Effective Throttle
        if self.omega_rad_s > 150.0 && ctrl.ignition_on && ctrl.fuel_pump_on {
            let err = (ctrl.idle_rpm_target - rpm) / 4000.0;
            self.idle_integral = (self.idle_integral + err * 0.14 * dt).clamp(0.015, 0.11);
        } else {
            self.idle_integral = 0.050;
        }
        let idle_cmd = if self.omega_rad_s > 100.0 {
            (self.idle_integral + ((ctrl.idle_rpm_target - rpm) / 4500.0) * 0.065).clamp(0.008, 0.12)
        } else {
            0.035
        };
        self.effective_throttle = requested_throttle.max(idle_cmd);

        // 3. Rev Limiter (hard ignition cut with subtle hysteresis)
        if rpm >= ctrl.rev_limit_rpm {
            self.rev_cut_timer = 0.022;
            self.rev_limiter_active = true;
        } else if self.rev_cut_timer > 0.0 {
            self.rev_cut_timer = (self.rev_cut_timer - dt).max(0.0);
            self.rev_limiter_active = true;
        } else {
            self.rev_limiter_active = false;
        }
        let fuel_cut = self.rev_cut_timer > 0.0 || self.shift_cut_timer > 0.0;
        let global_spark_cut = fuel_cut;
        if global_spark_cut {
            self.post_cut_timer = 0.08;
        } else if self.post_cut_timer > 0.0 {
            self.post_cut_timer = (self.post_cut_timer - dt).max(0.0);
        }

        // 4. Variable Intake Trumpet Length (75mm high-RPM to 135mm low/mid-RPM)
        let target_trumpet_m = if ctrl.trumpets_auto {
            let t = ((rpm - 11000.0) / 7500.0).clamp(0.0, 1.0);
            0.135 - t * (0.135 - 0.075)
        } else {
            (ctrl.manual_trumpet_mm * 0.001).clamp(0.075, 0.135)
        };
        // Hydraulic trumpet actuator slew rate
        let slew = (target_trumpet_m - self.trumpet_length_m).clamp(-0.25 * dt, 0.25 * dt);
        self.trumpet_length_m += slew;

        // Helmholtz / Quarter-Wave Intake Ram Resonance Factor
        // Total effective runner+trumpet acoustic length = 0.085 + trumpet_length_m
        let l_eff = 0.085 + self.trumpet_length_m;
        let round_trip_2 = 4.0 * l_eff / 348.0; // 2nd pressure wave reflection
        let open_dur = (240.0 / 360.0) * (60.0 / rpm.max(2500.0));
        let phase_err = ((open_dur - round_trip_2) / round_trip_2).abs();
        let ram_boost = if rpm > 5000.0 {
            1.0 + 0.30 * (-phase_err * phase_err * 7.5).exp() * ((rpm - 5000.0) / 14000.0).clamp(0.0, 1.0)
        } else {
            1.0
        };

        // 5. Intake Plenum Pressure Dynamics
        let throttle_area = 0.0000015 + 0.0135 * self.effective_throttle.powf(2.0);
        let dm_plenum_in = orifice_mass_flow(P_ATM, T_ATM, self.plenum_pressure_pa, T_ATM, throttle_area);
        // Plenum volume ~ 5.5 liters
        let mut dm_plenum_out = 0.0;

        // 6. Pneumatic Valve Return Float Check
        let req_n2_bar = 30.0 + 115.0 * (rpm / 19000.0).powi(2);
        let n2_deficit = (req_n2_bar - ctrl.n2_pressure_bar).max(0.0);
        let valve_float_fraction = if ctrl.n2_pressure_bar < 15.0 {
            // Completely unpressurized pneumatic springs: valves stay drooped open!
            0.35
        } else {
            (n2_deficit / 70.0).clamp(0.0, 0.35)
        };
        self.valve_float_active = valve_float_fraction > 0.015 && rpm > 1200.0;

        // 7. Advance Crank Angle & Step All 10 Cylinders
        let d_theta = self.omega_rad_s * dt;
        self.crank_angle_rad = wrap_720(self.crank_angle_rad + d_theta);

        let mut total_gas_torque = 0.0;
        let mut sum_intake_mass = 0.0;
        let mut heat_to_coolant = 0.0;

        // Spark advance (deg BTDC): ramps from 14 deg at idle up to 28 deg MBT at 19k RPM + ECU offset
        let base_adv_deg = 14.0 + 14.0 * ((rpm - 4000.0) / 15000.0).clamp(0.0, 1.0);
        let spark_adv_deg = (base_adv_deg + ctrl.spark_offset_deg).clamp(2.0, 48.0);
        let spark_angle_rad = (720.0 - spark_adv_deg).to_radians();

        for idx in 0..NUM_CYL {
            let rand_burn = 1.0 + 0.06 * (self.fast_noise() as f64);
            let bank = idx / 5;
            let cyl = &mut self.cylinders[idx];
            let theta = wrap_720(self.crank_angle_rad - cyl.firing_offset_rad);
            let prev_theta = cyl.prev_local_angle;
            cyl.prev_local_angle = theta;

            let (x, dx_dtheta, _) = slider_crank(theta);
            cyl.piston_x = x;
            let vol = CLEARANCE_VOL_M3 + PISTON_AREA_M2 * x;
            let dv = PISTON_AREA_M2 * dx_dtheta * d_theta;

            // Valve lifts (0..1)
            let in_lift = (intake_lift_norm(theta) + valve_float_fraction).min(1.0);
            let ex_lift = (exhaust_lift_norm(theta) + valve_float_fraction).min(1.0);
            cyl.intake_lift = in_lift;
            cyl.exhaust_lift = ex_lift;

            // Effective valve throat/curtain areas (2x 40.5mm downdraft intake, 2x 32.5mm exhaust)
            let a_in = in_lift * 0.00235;
            let a_ex = ex_lift * 0.00175;

            // Dynamic intake port pressure boosted by variable trumpet Helmholtz wave near IVC
            let p_port_in = self.plenum_pressure_pa * ram_boost;
            let p_runner = cyl.runner_gas.pressure_pa();
            let t_runner = cyl.runner_gas.temp_k();

            let mut dm_in = if a_in > 1e-7 {
                orifice_mass_flow(p_port_in, T_ATM + 12.0, cyl.pressure_pa, cyl.temp_k, a_in) * dt
            } else {
                0.0
            };
            let mut dm_ex = if a_ex > 1e-7 {
                orifice_mass_flow(cyl.pressure_pa, cyl.temp_k, p_runner, t_runner, a_ex) * dt
            } else {
                0.0
            };

            // Numerical safeguard near TDC overlap
            let max_step_mass = (cyl.air_mass * 0.25).max(2.0e-6);
            dm_in = dm_in.clamp(-max_step_mass, max_step_mass * 1.8);
            dm_ex = dm_ex.clamp(-max_step_mass, max_step_mass * 1.8);

            let m_old_track = cyl.air_mass.max(1e-6);
            if dm_in > 0.0 {
                cyl.fresh_air_mass += dm_in;
                cyl.intake_accum_mass += dm_in;
            } else if dm_in < 0.0 {
                let keep = (1.0 + dm_in / m_old_track).clamp(0.0, 1.0);
                cyl.fresh_air_mass *= keep;
            }
            if dm_ex > 0.0 {
                let keep = (1.0 - dm_ex / m_old_track).clamp(0.0, 1.0);
                cyl.fresh_air_mass *= keep;
            }
            cyl.fresh_air_mass = cyl.fresh_air_mass.min(m_old_track + dm_in.max(0.0));

            dm_plenum_out += dm_in / dt;

            // Fuel injection during intake stroke (cut cleanly during shift/rev cut for click-free seamless shifts)
            if dm_in > 0.0 && ctrl.fuel_pump_on && !fuel_cut {
                let target_afr = AFR_STOICH * ctrl.lambda_target.clamp(0.72, 1.25);
                cyl.fuel_mass += dm_in / target_afr;
            }

            // Spark Trigger at spark_angle_rad
            let crossed_spark = prev_theta < spark_angle_rad && theta >= spark_angle_rad;
            if crossed_spark {
                cyl.last_intake_mass = cyl.intake_accum_mass;
                cyl.intake_accum_mass = 0.0;

                if ctrl.ignition_on && ctrl.fuel_pump_on && !global_spark_cut && !self.valve_float_active {
                    cyl.combusting = true;
                    cyl.comb_progress = 0.0;
                    cyl.comb_start_rad = theta;
                    cyl.spark_timer = 0.0025;
                    cyl.spark_cut_this_cycle = false;

                    let max_burn_fuel = cyl.fresh_air_mass / AFR_STOICH;
                    let burn_fuel = cyl.fuel_mass.min(max_burn_fuel * 1.04);
                    let rich_bonus = if ctrl.lambda_target < 1.0 {
                        1.0 + 0.05 * (1.0 - ctrl.lambda_target)
                    } else {
                        1.0 - 0.35 * (ctrl.lambda_target - 1.0)
                    };
                    cyl.comb_energy_total =
                        (burn_fuel * Q_LHV_FUEL * 0.85 * rich_bonus * rand_burn).max(0.0);
                    cyl.fuel_mass = (cyl.fuel_mass - burn_fuel).max(0.0);
                    cyl.fresh_air_mass = (cyl.fresh_air_mass - burn_fuel * AFR_STOICH).max(0.0);
                    cyl.unburnt_fuel = 0.0;

                    let high_load = self.plenum_pressure_pa > 78_000.0;
                    let knock_risk = (ctrl.spark_offset_deg > 7.5 && high_load)
                        || (ctrl.lambda_target > 1.08 && high_load)
                        || (self.coolant_temp_c > 118.0 && high_load);
                    if knock_risk && rpm > 6000.0 {
                        cyl.knock_timer = 0.04;
                        self.knock_active_timer = 0.25;
                        self.knock_count += 1;
                        self.bore_health = (self.bore_health - 0.003).max(0.05);
                    }
                } else {
                    cyl.unburnt_fuel = cyl.fuel_mass;
                    cyl.fuel_mass = 0.0;
                    cyl.spark_cut_this_cycle = true;
                }
            }

            // Wiebe heat release (~66 deg burn duration -> MFB50 around ~7 deg ATDC)
            let mut dq_comb = 0.0;
            if cyl.combusting {
                let burn_dur = 1.15;
                let elapsed = wrap_720(theta - cyl.comb_start_rad);
                if elapsed < burn_dur {
                    let frac = (elapsed / burn_dur).clamp(0.0, 1.0);
                    let xb = 1.0 - (-5.0 * frac * frac * frac).exp();
                    let dxb = (xb - cyl.comb_progress).max(0.0);
                    cyl.comb_progress = xb;
                    dq_comb = cyl.comb_energy_total * dxb;
                } else {
                    let rem = (1.0 - cyl.comb_progress).max(0.0);
                    dq_comb = cyl.comb_energy_total * rem;
                    cyl.comb_progress = 1.0;
                    cyl.combusting = false;
                }
            }

            // 1st-Law Energy Balance
            let m_old = cyl.air_mass.max(1e-6);
            let m_new = (m_old + dm_in - dm_ex).max(1e-6);

            let h_in_spec = GAMMA_AIR * CV_GAS * (if dm_in >= 0.0 { T_ATM + 12.0 } else { cyl.temp_k });
            let h_ex_spec = GAMMA_AIR * CV_GAS * (if dm_ex >= 0.0 { cyl.temp_k } else { t_runner });

            let area_wall = 2.0 * PISTON_AREA_M2 + PI * BORE_M * x;
            let h_conv = 120.0 + 11.0 * (rpm / 1000.0);
            let t_wall = self.coolant_temp_c + 273.15 + 75.0;
            let dq_wall = h_conv * area_wall * (cyl.temp_k - t_wall) * dt;
            heat_to_coolant += dq_wall.max(0.0);

            let u_old = m_old * CV_GAS * cyl.temp_k;
            let work = cyl.pressure_pa * dv;
            let u_new = u_old + dq_comb - dq_wall - work + dm_in * h_in_spec - dm_ex * h_ex_spec;

            cyl.air_mass = m_new;
            cyl.temp_k = (u_new / (m_new * CV_GAS)).clamp(240.0, 3400.0);
            cyl.pressure_pa = (m_new * R_AIR * cyl.temp_k / vol).clamp(12_000.0, 1.4e7);

            // Gas-dynamic runner & collector flow (ported from engine-sim CombustionChamber + ExhaustSystem)
            cyl.runner_gas.add_flow_from_cylinder(dm_ex, cyl.temp_k, dt);
            if cyl.unburnt_fuel > 1e-8 && self.egt_c > 350.0 {
                // Smooth afterburn oxidation in hot primary header (continuous heat release prevents derivative click)
                let d_burn = cyl.unburnt_fuel * (12.0 * dt).min(1.0);
                cyl.runner_gas.e_internal += d_burn * Q_LHV_FUEL * 0.08;
                cyl.unburnt_fuel = (cyl.unburnt_fuel - d_burn).max(0.0);
            }
            cyl.runner_gas.dissipate_excess_velocity();
            cyl.runner_gas.flow_to_node(&mut self.collectors[bank], dt, 0.00185);
            cyl.runner_gas.dissipate_excess_velocity();
            cyl.runner_gas.update_velocity(dt, 0.75);

            if cyl.spark_timer > 0.0 {
                cyl.spark_timer = (cyl.spark_timer - dt).max(0.0);
            }
            if cyl.knock_timer > 0.0 {
                cyl.knock_timer = (cyl.knock_timer - dt).max(0.0);
            }

            total_gas_torque += (cyl.pressure_pa - P_ATM) * PISTON_AREA_M2 * dx_dtheta;
            sum_intake_mass += cyl.last_intake_mass;

            if idx == 0 {
                let pv_idx = (((theta / FOUR_PI) * PV_POINTS as f64) as usize).min(PV_POINTS - 1);
                self.pv_trace[pv_idx] = ((vol * 1e6) as f32, (cyl.pressure_pa * 1e-5) as f32);
            }
        }

        for col in &mut self.collectors {
            col.dissipate_excess_velocity();
            col.flow_to_atmosphere(dt, 0.0052);
            col.dissipate_excess_velocity();
            col.update_velocity(dt, 0.85);
        }

        // Update Plenum pressure (allow true high-vacuum down to 0.05 bar on idle/overrun)
        let vol_plenum = 0.0055;
        let dm_plenum_net = (dm_plenum_in - dm_plenum_out) * dt;
        let rho_plenum = self.plenum_pressure_pa / (R_AIR * T_ATM);
        let new_rho = (rho_plenum + dm_plenum_net / vol_plenum).clamp(0.04, 1.8);
        self.plenum_pressure_pa = (new_rho * R_AIR * T_ATM).clamp(4_500.0, 130_000.0);

        // 8. Mechanical Friction, Oil Viscosity & Authentic F1 Cold-Seize Tolerance Lock
        let min_temp = self.coolant_temp_c.min(self.oil_temp_c);
        let seize_torque = if min_temp < 58.0 {
            (58.0 - min_temp) * 6.8
        } else {
            0.0
        };
        self.cold_seized = seize_torque > 25.0 && ctrl.starter_active && rpm < 450.0;
        if self.cold_seized {
            self.bore_health = (self.bore_health - 0.015 * dt).max(0.10);
        }

        // High-RPM pumping + bearing friction (scales with viscosity and RPM^2 reciprocating load; ~2.4 bar FMEP @ 19k)
        let oil_visc_factor = 1.0 + ((70.0 - self.oil_temp_c).max(0.0) / 50.0) * 1.1;
        let bore_damage_drag = 1.0 + (1.0 - self.bore_health) * 1.5;
        let base_friction = (9.0 + 0.9 * (rpm / 1000.0) + 0.088 * (rpm / 1000.0).powi(2))
            * oil_visc_factor
            * bore_damage_drag;

        let friction_torque = if self.omega_rad_s.abs() > 0.5 {
            (base_friction + seize_torque) * self.omega_rad_s.signum()
        } else {
            (base_friction + seize_torque) * (self.omega_rad_s / 0.5)
        };

        // External Pneumatic Starter Gun torque
        let starter_torque = if ctrl.starter_active {
            // Pneumatic air starter delivers ~145 Nm up to ~3,600 RPM
            (145.0 * (1.0 - rpm / 4000.0).clamp(0.0, 1.0)).max(0.0)
        } else {
            0.0
        };

        // Indicated / Brake Engine Torque
        let brake_engine_torque = total_gas_torque - friction_torque;

        // 9. Drivetrain / Dyno / Vehicle Dynamics Integration
        let mut net_load_torque = 0.0;
        let mut eff_inertia = ENGINE_INERTIA;

        match ctrl.bench_mode {
            BenchMode::FreeRev => {
                // Light carbon clutch inertia only
                self.car_speed_ms *= (-0.15 * dt).exp();
            }
            BenchMode::DynoManualBrake => {
                let max_brake_nm = 520.0 * ctrl.dyno_brake_load.clamp(0.0, 1.0);
                net_load_torque = max_brake_nm * (rpm / 6000.0).clamp(0.0, 1.5);
            }
            BenchMode::DynoRpmHold => {
                eff_inertia += 0.28;
                let target_omega = ctrl.dyno_hold_rpm * 2.0 * PI / 60.0;
                let err = self.omega_rad_s - target_omega;
                net_load_torque = (err * 45.0).clamp(-350.0, 800.0);
            }
            BenchMode::DynoSweep => {
                eff_inertia += 0.28;
                if self.dyno_sweep_running {
                    let target_omega = self.dyno_sweep_rpm * 2.0 * PI / 60.0;
                    let err = self.omega_rad_s - target_omega;
                    net_load_torque = (err * 45.0).clamp(-350.0, 800.0);
                }
            }
            BenchMode::TrackManualGearbox | BenchMode::MonzaHotlapAuto => {
                let g = self.last_gear.clamp(1, 7);
                let total_ratio = GEAR_RATIOS[g - 1] * FINAL_DRIVE;
                let wheel_to_crank = WHEEL_RADIUS_M / total_ratio;

                // Aero drag + rolling resistance reflected to crank
                let f_aero = 0.5 * 1.20 * CAR_CDA * self.car_speed_ms * self.car_speed_ms;
                let f_roll = 180.0;
                let f_total = f_aero + f_roll + (brake_torque_car / WHEEL_RADIUS_M);

                // Smooth raised-sine dog-box synchronizer during gear change vs locked driveline
                if self.shift_sync_timer > 0.0 {
                    self.shift_sync_timer = (self.shift_sync_timer - dt).max(0.0);
                    let u = 1.0 - (self.shift_sync_timer / 0.036).clamp(0.0, 1.0);
                    let env = (u * PI).sin().powi(2);
                    let target_omega = (self.car_speed_ms / wheel_to_crank).clamp(350.0, 2150.0);
                    let k_sync = 1.2 + 9.0 * env + 8.0 * u * u;
                    let sync_tau = ((self.omega_rad_s - target_omega) * k_sync).clamp(-900.0, 900.0);
                    net_load_torque = sync_tau;
                    let car_accel = (sync_tau / wheel_to_crank - f_total) / CAR_MASS_KG;
                    self.car_speed_ms = (self.car_speed_ms + car_accel * dt).max(0.0);
                } else {
                    // Progressive carbon launch clutch in 1st gear until driveline locks up
                    let target_wheel_spd = self.omega_rad_s * wheel_to_crank;
                    if g == 1 && (target_wheel_spd - self.car_speed_ms).abs() > 0.5 && self.car_speed_ms < 36.0 {
                        let engage = ((rpm - 4300.0) / 3500.0).clamp(0.0, 1.0);
                        let clutch_cap = 420.0 * engage;
                        let slip = target_wheel_spd - self.car_speed_ms;
                        let clutch_torque = (slip * 35.0).clamp(-clutch_cap, clutch_cap);
                        net_load_torque = clutch_torque;
                        let car_accel = (clutch_torque / wheel_to_crank - f_total) / CAR_MASS_KG;
                        self.car_speed_ms = (self.car_speed_ms + car_accel * dt).max(0.0);
                    } else {
                        let reflected_inertia = CAR_MASS_KG * wheel_to_crank * wheel_to_crank;
                        eff_inertia += reflected_inertia;
                        net_load_torque = f_total * wheel_to_crank;
                        let residual = target_wheel_spd - self.car_speed_ms;
                        self.car_speed_ms = (self.car_speed_ms + residual * (220.0 * dt).min(1.0)).max(0.0);
                    }
                }
                self.monza_dist_m = (self.monza_dist_m + self.car_speed_ms * dt) % 5793.0;
            }
        }

        let net_torque = brake_engine_torque + starter_torque - net_load_torque;
        self.omega_rad_s = (self.omega_rad_s + (net_torque / eff_inertia) * dt).clamp(0.0, 2350.0);
        rpm = self.omega_rad_s * 60.0 / (2.0 * PI);

        // 10. Thermal & Exhaust Gas Temperature (EGT) Simulation
        let heater_coolant = if ctrl.coolant_heater_on { 7.2 } else { 0.0 };
        let heater_oil = if ctrl.oil_heater_on { 7.0 } else { 0.0 };
        let power_kw = (brake_engine_torque * self.omega_rad_s).max(0.0) * 0.001;

        // Test cell cooling loop regulates near 88C unless pushed hard continuously
        let radiator_cool = (self.coolant_temp_c - 86.0).max(0.0) * 0.14;
        let ambient_cool_c = (self.coolant_temp_c - 22.0) * 0.012;
        let d_coolant = (heat_to_coolant * 0.00012 + heater_coolant * (1.0 - (self.coolant_temp_c / 85.0).min(1.0)) * 2.2
            - radiator_cool
            - ambient_cool_c)
            * dt;
        self.coolant_temp_c = (self.coolant_temp_c + d_coolant).clamp(20.0, 140.0);

        let oil_cool = (self.oil_temp_c - self.coolant_temp_c) * 0.08 + (self.oil_temp_c - 22.0) * 0.01;
        let oil_fric_heat = (friction_torque * self.omega_rad_s).max(0.0) * 0.00015;
        let d_oil = (oil_fric_heat + heater_oil * (1.0 - (self.oil_temp_c / 85.0).min(1.0)) * 2.2 - oil_cool) * dt;
        self.oil_temp_c = (self.oil_temp_c + d_oil).clamp(20.0, 145.0);

        let target_egt = if ctrl.ignition_on && ctrl.fuel_pump_on && rpm > 1200.0 {
            380.0 + (power_kw / 700.0) * 580.0 + (rpm / 19000.0) * 140.0
        } else {
            25.0
        };
        self.egt_c += (target_egt - self.egt_c) * 2.2 * dt;

        if self.knock_active_timer > 0.0 {
            self.knock_active_timer = (self.knock_active_timer - dt).max(0.0);
        }

        // 11. Exact 720-deg Cycle-Mean Shaft Work Torque & Smooth Dyno Curve Recording
        if d_theta > 0.0 {
            self.interval_work_accum += brake_engine_torque * d_theta;
            self.interval_angle_accum += d_theta;
            let interval_rad = FOUR_PI / NUM_CYL as f64;
            if self.interval_angle_accum >= interval_rad {
                self.firing_work_buf[self.firing_slot] = self.interval_work_accum;
                self.firing_angle_buf[self.firing_slot] = self.interval_angle_accum;
                self.firing_slot = (self.firing_slot + 1) % NUM_CYL;
                self.interval_work_accum = 0.0;
                self.interval_angle_accum = 0.0;

                let sum_w: f64 = self.firing_work_buf.iter().sum();
                let sum_a: f64 = self.firing_angle_buf.iter().sum();
                if sum_a > 1e-6 {
                    self.cycle_mean_torque_nm = sum_w / sum_a;
                }
            }
        }

        let torque_target = if rpm > 300.0 {
            self.cycle_mean_torque_nm
        } else {
            0.0
        };
        let filter_alpha = (90.0 * dt).min(1.0);
        self.filtered_torque_nm += (torque_target - self.filtered_torque_nm) * filter_alpha;
        let inst_bhp = (self.filtered_torque_nm * self.omega_rad_s) / 745.69987;
        self.filtered_power_bhp += (inst_bhp - self.filtered_power_bhp) * filter_alpha;

        let theoretical_air = 1.184 * DISP_VOL_M3 * NUM_CYL as f64;
        let ve = (sum_intake_mass / theoretical_air) * 100.0;
        self.filtered_ve += (ve - self.filtered_ve) * filter_alpha;

        let sweep_settled = ctrl.bench_mode != BenchMode::DynoSweep || self.dyno_sweep_rpm > 5700.0;
        if rpm >= DYNO_MIN_RPM as f64
            && rpm <= DYNO_MAX_RPM as f64
            && self.effective_throttle > 0.90
            && !global_spark_cut
            && self.post_cut_timer <= 0.0
            && self.shift_sync_timer <= 0.0
            && self.firing_angle_buf[NUM_CYL - 1] > 0.0
            && sweep_settled
        {
            let bin = (((rpm - DYNO_MIN_RPM as f64) / DYNO_STEP_RPM as f64).round() as usize)
                .min(DYNO_BINS - 1);
            let t_nm = self.filtered_torque_nm.max(0.0) as f32;
            let p_bhp = self.filtered_power_bhp.max(0.0) as f32;
            if !self.dyno_curve[bin].recorded {
                self.dyno_curve[bin].torque_nm = t_nm;
                self.dyno_curve[bin].power_bhp = p_bhp;
                self.dyno_curve[bin].recorded = true;
            } else {
                self.dyno_curve[bin].torque_nm = self.dyno_curve[bin].torque_nm * 0.85 + t_nm * 0.15;
                self.dyno_curve[bin].power_bhp = self.dyno_curve[bin].power_bhp * 0.85 + p_bhp * 0.15;
            }
        }
    }

    pub fn snapshot(&self, ctrl: &SharedControls) -> TelemetrySnapshot {
        let rpm = (self.omega_rad_s * 60.0 / (2.0 * PI)).max(0.0);
        let req_n2_bar = 30.0 + 115.0 * (rpm / 19000.0).powi(2);

        let mut cyl_snaps = [CylinderSnapshot::default(); NUM_CYL];
        for i in 0..NUM_CYL {
            let c = &self.cylinders[i];
            let theta = wrap_720(self.crank_angle_rad - c.firing_offset_rad);
            let phase = if theta < PI {
                StrokePhase::Power
            } else if theta < 2.0 * PI {
                StrokePhase::Exhaust
            } else if theta < 3.0 * PI {
                StrokePhase::Intake
            } else {
                StrokePhase::Compression
            };
            let comb_vis = if c.combusting {
                1.0
            } else {
                ((c.temp_k - 900.0) / 1700.0).clamp(0.0, 0.85)
            };
            cyl_snaps[i] = CylinderSnapshot {
                local_angle_rad: theta,
                piston_norm: (c.piston_x / STROKE_M).clamp(0.0, 1.0),
                pressure_bar: (c.pressure_pa * 1e-5).max(0.0),
                temp_k: c.temp_k,
                intake_lift_norm: c.intake_lift,
                exhaust_lift_norm: c.exhaust_lift,
                combustion_intensity: comb_vis,
                spark_firing: c.spark_timer > 0.0,
                fuel_injecting: ctrl.fuel_pump_on && theta > 6.8 && theta < 9.5 && rpm > 80.0,
                phase,
            };
        }

        let gear = if matches!(
            ctrl.bench_mode,
            BenchMode::TrackManualGearbox | BenchMode::MonzaHotlapAuto
        ) {
            self.last_gear
        } else {
            0
        };

        // Re-order oscilloscope buffers chronologically starting at scope_ptr
        let mut scope_l = [0.0f32; SCOPE_LEN];
        let mut scope_r = [0.0f32; SCOPE_LEN];
        let mut scope_a = [0.0f32; SCOPE_LEN];
        for i in 0..SCOPE_LEN {
            let idx = (self.scope_ptr + i) % SCOPE_LEN;
            scope_l[i] = self.scope_left[idx];
            scope_r[i] = self.scope_right[idx];
            scope_a[i] = self.scope_audio[idx];
        }

        TelemetrySnapshot {
            crank_angle_rad: self.crank_angle_rad,
            rpm,
            torque_nm: self.filtered_torque_nm,
            power_bhp: self.filtered_power_bhp,
            plenum_pressure_bar: self.plenum_pressure_pa * 1e-5,
            ve_percent: self.filtered_ve,
            trumpet_length_mm: self.trumpet_length_m * 1000.0,
            coolant_temp_c: self.coolant_temp_c,
            oil_temp_c: self.oil_temp_c,
            egt_c: self.egt_c,
            bore_health: self.bore_health,
            knock_count: self.knock_count,
            knock_active: self.knock_active_timer > 0.0,
            valve_float_active: self.valve_float_active,
            cold_seized: self.cold_seized,
            rev_limiter_active: self.rev_limiter_active,
            is_running: rpm > 1500.0 && ctrl.ignition_on && ctrl.fuel_pump_on,
            gear,
            speed_kmh: self.car_speed_ms * 3.6,
            monza_dist_m: self.monza_dist_m,
            monza_sector_name: self.monza_sector_name,
            dyno_sweep_running: self.dyno_sweep_running,
            effective_throttle: self.effective_throttle,
            required_n2_bar: req_n2_bar,
            cylinders: cyl_snaps,
            pv_trace: self.pv_trace,
            dyno_curve: self.dyno_curve,
            scope_left: scope_l,
            scope_right: scope_r,
            scope_audio: scope_a,
        }
    }
}

#[inline]
pub fn wrap_720(a: f64) -> f64 {
    let mut x = a % FOUR_PI;
    if x < 0.0 {
        x += FOUR_PI;
    }
    x
}

/// Exact slider-crank position from TDC (m), dx/dtheta (m/rad), and crankpin offset
#[inline]
pub fn slider_crank(theta: f64) -> (f64, f64, f64) {
    let r = CRANK_RADIUS_M;
    let l = CONROD_M;
    let lam = r / l;
    let s = theta.sin();
    let c = theta.cos();
    let root = (1.0 - lam * lam * s * s).max(1e-6).sqrt();
    let x = r * (1.0 - c) + l * (1.0 - root);
    let dx_dtheta = r * s * (1.0 + (lam * c) / root);
    (x, dx_dtheta, root)
}

/// Smooth raised-cosine F1 intake cam profile (0..1)
/// Cycle 0=Combustion TDC, 180=BDC, 360=Overlap TDC, 540=BDC, 720=Combustion TDC
#[inline]
fn intake_lift_norm(theta: f64) -> f64 {
    // Opens ~325 deg (35 deg BTDC overlap), closes ~595 deg (55 deg ABDC)
    let open = 325.0_f64.to_radians();
    let close = 595.0_f64.to_radians();
    if theta <= open || theta >= close {
        return 0.0;
    }
    let u = (theta - open) / (close - open);
    let s = (u * PI).sin();
    s * s
}

/// Smooth raised-cosine F1 exhaust cam profile (0..1)
#[inline]
fn exhaust_lift_norm(theta: f64) -> f64 {
    // Opens ~128 deg (52 deg BBDC), closes ~392 deg (32 deg ATDC overlap)
    let open = 128.0_f64.to_radians();
    let close = 392.0_f64.to_radians();
    if theta <= open || theta >= close {
        return 0.0;
    }
    let u = (theta - open) / (close - open);
    let s = (u * PI).sin();
    s * s
}

/// Compressible isentropic orifice mass flow (kg/s) from upstream (p1, t1) to downstream (p2, t2)
#[inline]
fn orifice_mass_flow(p1: f64, t1: f64, p2: f64, t2: f64, area: f64) -> f64 {
    if area <= 1e-9 || (p1 - p2).abs() < 1.0 {
        return 0.0;
    }
    let (p_up, t_up, p_dn, sign) = if p1 >= p2 {
        (p1, t1.max(200.0), p2, 1.0)
    } else {
        (p2, t2.max(200.0), p1, -1.0)
    };
    let pr = (p_dn / p_up).clamp(0.05, 1.0);
    let g = GAMMA_AIR;
    let crit_pr = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
    let cd = 0.72;
    let flow = if pr <= crit_pr {
        // Choked sonic flow
        let choked_const = (g / R_AIR).sqrt() * (2.0 / (g + 1.0)).powf((g + 1.0) / (2.0 * (g - 1.0)));
        cd * area * p_up * choked_const / t_up.sqrt()
    } else {
        let term = (pr.powf(2.0 / g) - pr.powf((g + 1.0) / g)).max(0.0);
        let sub_const = ((2.0 * g) / (R_AIR * (g - 1.0))).sqrt();
        cd * area * p_up * sub_const * term.sqrt() / t_up.sqrt()
    };
    sign * flow
}

/// Automated Monza Qualifying Lap Speed/Gear Controller (5,793m track)
fn monza_driver(
    dist_m: f64,
    speed_ms: f64,
    current_gear: usize,
) -> (f64, f64, usize, &'static str) {
    // (End distance, target apex speed m/s, sector label)
    const SEGMENTS: &[(f64, f64, &str)] = &[
        (980.0, 99.0, "Main Straight"),
        (1140.0, 21.0, "Variante del Rettifilo"),
        (1820.0, 86.0, "Curva Grande"),
        (1990.0, 29.0, "Variante della Roggia"),
        (2520.0, 74.0, "Approach to Lesmo 1"),
        (2680.0, 48.0, "Curva di Lesmo 1"),
        (3080.0, 72.0, "Approach to Lesmo 2"),
        (3250.0, 46.0, "Curva di Lesmo 2"),
        (4020.0, 94.0, "Serraglio Straight"),
        (4250.0, 45.0, "Variante Ascari"),
        (5120.0, 96.0, "Back Straight"),
        (5420.0, 56.0, "Curva Parabolica"),
        (5793.0, 99.0, "Parabolica Exit"),
    ];

    let mut cur_idx = 0;
    for (i, &(end_d, _, _)) in SEGMENTS.iter().enumerate() {
        if dist_m < end_d {
            cur_idx = i;
            break;
        }
    }
    let (end_d, seg_speed, label) = SEGMENTS[cur_idx];
    let next_idx = (cur_idx + 1) % SEGMENTS.len();
    let next_speed = SEGMENTS[next_idx].1;
    let dist_to_next = (end_d - dist_m).max(1.0);

    // Compute braking distance at 4.3G (~42 m/s^2) F1 carbon brakes
    let need_brake_for_next = if speed_ms > next_speed {
        let req_dist = (speed_ms * speed_ms - next_speed * next_speed) / (2.0 * 41.0);
        dist_to_next <= req_dist + 18.0
    } else {
        false
    };

    let (tps, brake_nm) = if speed_ms > seg_speed + 2.5 || need_brake_for_next {
        (0.0, 4800.0)
    } else if speed_ms < seg_speed - 1.5 {
        (1.0, 0.0)
    } else {
        (0.45, 0.0)
    };

    // Hysteretic sequential F1 paddle shifting using road-coupled wheel RPM
    let mut gear = current_gear.clamp(1, 7);
    let wheel_rpm = (speed_ms / WHEEL_RADIUS_M) * GEAR_RATIOS[gear - 1] * FINAL_DRIVE * 60.0 / (2.0 * PI);
    if tps > 0.6 && wheel_rpm >= 18_600.0 && gear < 7 {
        gear += 1;
    } else if gear > 1 && wheel_rpm < 13_400.0 {
        let lower_ratio = GEAR_RATIOS[gear - 2] / GEAR_RATIOS[gear - 1];
        if wheel_rpm * lower_ratio <= 17_800.0 {
            gear -= 1;
        }
    }

    (tps, brake_nm, gear, label)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cold_seizure_blocks_cold_cranking() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        ctrl.n2_pressure_bar = 165.0;
        ctrl.fuel_pump_on = true;
        ctrl.ignition_on = true;
        ctrl.starter_active = true;

        // Step 0.5 seconds at room temperature (22C)
        for _ in 0..24_000 {
            sim.step_sample(&mut ctrl);
        }
        let snap = sim.snapshot(&ctrl);
        assert!(snap.rpm < 400.0, "Expected cold seize stall, got {} RPM", snap.rpm);
        assert!(snap.cold_seized, "Expected cold_seized flag to be true");
    }

    #[test]
    fn quick_start_maintains_stable_f1_idle() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        sim.quick_start(&mut ctrl);

        // Run for 1.5 seconds at 0% throttle
        for _ in 0..72_000 {
            sim.step_sample(&mut ctrl);
        }
        let snap = sim.snapshot(&ctrl);
        assert!(
            snap.rpm > 3500.0 && snap.rpm < 4700.0,
            "Expected stable F1 idle around 4000 RPM, got {} RPM",
            snap.rpm
        );
        assert!(snap.is_running);
    }

    #[test]
    fn full_throttle_19k_produces_v10_power_and_valid_audio() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        sim.quick_start(&mut ctrl);
        ctrl.bench_mode = BenchMode::DynoRpmHold;
        ctrl.dyno_hold_rpm = 19_000.0;
        ctrl.throttle = 1.0;
        sim.omega_rad_s = 19_000.0 * 2.0 * PI / 60.0;

        let mut max_p_bar = 0.0f64;
        let mut energy = 0.0f64;
        for _ in 0..48_000 {
            let (l, r) = sim.step_sample(&mut ctrl);
            assert!(l.is_finite() && r.is_finite(), "Audio produced non-finite sample");
            energy += (l as f64 * l as f64 + r as f64 * r as f64) * 0.5;
            max_p_bar = max_p_bar.max(sim.cylinders[0].pressure_pa * 1e-5);
        }
        let snap = sim.snapshot(&ctrl);
        assert!(
            snap.power_bhp > 780.0 && snap.power_bhp < 1050.0,
            "Expected ~850-950 BHP @ 19k RPM, got {} BHP",
            snap.power_bhp
        );
        assert!(max_p_bar > 65.0, "Expected peak combustion > 65 bar, got {} bar", max_p_bar);
        assert!(energy > 10.0, "Expected energetic exhaust audio output");
    }

    #[test]
    fn low_n2_pressure_triggers_valve_float_and_power_loss() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        sim.quick_start(&mut ctrl);
        ctrl.bench_mode = BenchMode::DynoRpmHold;
        ctrl.dyno_hold_rpm = 18_500.0;
        ctrl.throttle = 1.0;
        ctrl.n2_pressure_bar = 40.0; // Too low for 18.5k RPM!
        sim.omega_rad_s = 18_500.0 * 2.0 * PI / 60.0;

        for _ in 0..24_000 {
            sim.step_sample(&mut ctrl);
        }
        let snap = sim.snapshot(&ctrl);
        assert!(snap.valve_float_active, "Expected valve float at low N2 pressure");
        assert!(
            snap.power_bhp < 550.0,
            "Expected heavy power loss under valve float, got {} BHP",
            snap.power_bhp
        );
    }

    #[test]
    fn warm_manual_starter_sequence_starts_engine() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        ctrl.coolant_heater_on = true;
        ctrl.oil_heater_on = true;
        ctrl.n2_pressure_bar = 165.0;
        ctrl.fuel_pump_on = true;
        ctrl.ignition_on = true;

        // Pre-heat fluids for 10 simulated seconds until > 65C
        for _ in 0..480_000 {
            sim.step_sample(&mut ctrl);
        }
        assert!(sim.coolant_temp_c > 65.0 && sim.oil_temp_c > 65.0);

        // Engage pneumatic starter for 0.6s, then release
        ctrl.starter_active = true;
        for _ in 0..28_800 {
            sim.step_sample(&mut ctrl);
        }
        ctrl.starter_active = false;
        for _ in 0..48_000 {
            sim.step_sample(&mut ctrl);
        }
        let snap = sim.snapshot(&ctrl);
        assert!(
            snap.is_running && snap.rpm > 3200.0 && snap.rpm < 4800.0,
            "Expected manual warm startup into idle, got {} RPM (running={})",
            snap.rpm,
            snap.is_running
        );
    }

    #[test]
    fn dyno_sweep_and_monza_lap_accelerate_cleanly() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        sim.quick_start(&mut ctrl);
        ctrl.start_dyno_sweep = true;

        // Run ~7 seconds for full 5.5k -> 19.5k dyno sweep
        for _ in 0..336_000 {
            sim.step_sample(&mut ctrl);
        }
        let snap = sim.snapshot(&ctrl);
        let recorded_bins = snap.dyno_curve.iter().filter(|d| d.recorded).count();
        assert!(
            recorded_bins >= 35,
            "Expected full dyno curve bins recorded, got {}",
            recorded_bins
        );
        // Verify smooth dyno curve (no sawtooth firing-ripple spikes between adjacent 250-RPM bins)
        let mut prev_bhp: Option<f32> = None;
        for pt in &snap.dyno_curve {
            if pt.recorded && pt.rpm >= 7000.0 {
                if let Some(pb) = prev_bhp {
                    let jump = (pt.power_bhp - pb).abs();
                    assert!(
                        jump < 45.0,
                        "Dyno curve jump too large at {} RPM: {:.1} -> {:.1} BHP",
                        pt.rpm,
                        pb,
                        pt.power_bhp
                    );
                }
                prev_bhp = Some(pt.power_bhp);
            }
        }

        // Switch to Monza qualifying lap for 8 seconds -> should reach high gear & > 260 km/h
        ctrl.bench_mode = BenchMode::MonzaHotlapAuto;
        let mut max_kmh = 0.0f64;
        for _ in 0..384_000 {
            sim.step_sample(&mut ctrl);
            max_kmh = max_kmh.max(sim.car_speed_ms * 3.6);
        }
        assert!(
            max_kmh > 260.0,
            "Expected Monza main straight acceleration > 260 km/h, reached {:.1} km/h",
            max_kmh
        );
    }

    #[test]
    fn upshift_under_full_throttle_has_no_acoustic_click_spike() {
        let mut sim = EngineSim::new(48_000.0);
        let mut ctrl = SharedControls::default();
        sim.quick_start(&mut ctrl);
        ctrl.bench_mode = BenchMode::TrackManualGearbox;
        ctrl.requested_gear = 1;
        ctrl.throttle = 1.0;

        // Bring engine + chassis cleanly up to ~18,600 RPM in 1st gear
        let wheel_to_crank_1 = WHEEL_RADIUS_M / (GEAR_RATIOS[0] * FINAL_DRIVE);
        sim.omega_rad_s = 18_600.0 * 2.0 * PI / 60.0;
        sim.car_speed_ms = sim.omega_rad_s * wheel_to_crank_1;

        for _ in 0..12_000 {
            sim.step_sample(&mut ctrl);
        }

        let (mut prev_l, mut prev_r) = sim.step_sample(&mut ctrl);
        let mut max_delta = 0.0_f32;

        // Upshift 1 -> 2, run 0.35s, then upshift 2 -> 3 and inspect sample-to-sample waveform deltas
        ctrl.requested_gear = 2;
        for step in 0..33_600 {
            if step == 16_800 {
                ctrl.requested_gear = 3;
            }
            let (l, r) = sim.step_sample(&mut ctrl);
            let dl = (l - prev_l).abs();
            let dr = (r - prev_r).abs();
            max_delta = max_delta.max(dl).max(dr);
            prev_l = l;
            prev_r = r;
        }
        assert!(
            max_delta < 0.35,
            "Expected click-free upshift audio delta < 0.35, got {:.3}",
            max_delta
        );
    }
}
