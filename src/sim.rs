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

    // Audio
    pub master_volume: f32,
    pub muted: bool,
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

            master_volume: 0.65,
            muted: false,
        }
    }
}

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
    ex_flow_pulse: f64,
    in_flow_pulse: f64,
    intake_lift: f64,
    exhaust_lift: f64,
    piston_x: f64,
    spark_cut_this_cycle: bool,
}

impl Cylinder {
    fn new(firing_offset_rad: f64) -> Self {
        let vol = CLEARANCE_VOL_M3;
        let air_mass = (P_ATM * vol) / (R_AIR * T_ATM);
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
            ex_flow_pulse: 0.0,
            in_flow_pulse: 0.0,
            intake_lift: 0.0,
            exhaust_lift: 0.0,
            piston_x: 0.0,
            spark_cut_this_cycle: false,
        }
    }
}

/// 1D Acoustic Waveguide for one 5-into-1 bank exhaust manifold & tailpipe
struct BankWaveguide {
    pri_bufs: [[f32; 128]; 5],
    pri_ptrs: [usize; 5],
    pri_delays: [usize; 5],
    tail_buf: [f32; 256],
    tail_ptr: usize,
    tail_delay: usize,
    tail_lp: f32,
    prev_col: f32,
    pop_env: f32,
}

impl BankWaveguide {
    fn new(sample_rate: f64) -> Self {
        // Slight realistic primary length stagger (0.39m .. 0.44m) gives authentic 5-into-1 F1 rasp
        let pri_lengths = [0.395, 0.415, 0.435, 0.425, 0.405];
        let c_ex = 560.0; // speed of sound in hot F1 exhaust gas (~800 C)
        let mut pri_delays = [24usize; 5];
        for i in 0..5 {
            let d = ((pri_lengths[i] / c_ex) * sample_rate).round() as usize;
            pri_delays[i] = d.clamp(6, 120);
        }
        let tail_delay = (((0.82 / c_ex) * sample_rate).round() as usize).clamp(16, 240);
        Self {
            pri_bufs: [[0.0; 128]; 5],
            pri_ptrs: [0; 5],
            pri_delays,
            tail_buf: [0.0; 256],
            tail_ptr: 0,
            tail_delay,
            tail_lp: 0.0,
            prev_col: 0.0,
            pop_env: 0.0,
        }
    }

    fn step(&mut self, cyl_drives: [f32; 5], pop_noise: f32) -> f32 {
        let mut collector_sum = 0.0f32;
        for c in 0..5 {
            let len = self.pri_delays[c];
            let p = self.pri_ptrs[c];
            let read_idx = (p + 128 - len) & 127;
            let delayed = self.pri_bufs[c][read_idx];

            // Primary pipe reflection + fresh blowdown pulse injection
            let wave = cyl_drives[c] + delayed * 0.42;
            self.pri_bufs[c][p] = wave.clamp(-6.0, 6.0);
            self.pri_ptrs[c] = (p + 1) & 127;
            collector_sum += delayed;
        }

        // Unburnt fuel overrun / shift crackle
        collector_sum += self.pop_env * pop_noise;
        self.pop_env *= 0.992;

        // 5-into-1 Tailpipe Waveguide with open-end acoustic reflection
        let tp = self.tail_ptr;
        let t_read = (tp + 256 - self.tail_delay) & 255;
        let tail_del = self.tail_buf[t_read];
        self.tail_lp += 0.35 * (tail_del - self.tail_lp);

        let col_wave = (collector_sum * 0.45 - 0.66 * self.tail_lp).clamp(-8.0, 8.0);
        self.tail_buf[tp] = col_wave;
        self.tail_ptr = (tp + 1) & 255;

        // Open pipe end radiation (high-pass derivative + warm low-mid body + shock steepening)
        let diff = col_wave - self.prev_col;
        self.prev_col = col_wave;
        let raw_rad = diff * 2.4 + col_wave * 0.65;
        // Supersonic exhaust shock steepening (asymmetric saturation creates F1 brassiness)
        let steep = (raw_rad * 1.65 + 0.16 * raw_rad * raw_rad).tanh();
        steep
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
    rev_limiter_active: bool,
    shift_cut_timer: f64,
    blip_timer: f64,
    shift_cooldown: f64,
    last_gear: usize,

    // Filtered telemetry
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

    // Audio DSP state
    left_wg: BankWaveguide,
    right_wg: BankWaveguide,
    intake_buf: [f32; 256],
    intake_ptr: usize,
    intake_lp: f32,
    dc_prev_in_l: f32,
    dc_prev_out_l: f32,
    dc_prev_in_r: f32,
    dc_prev_out_r: f32,
    air_lp_l: f32,
    air_lp_r: f32,
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

        let cylinders = std::array::from_fn(|i| Cylinder::new(firing_offsets[i]));
        let mut dyno_curve = [DynoPoint::default(); DYNO_BINS];
        for (i, pt) in dyno_curve.iter_mut().enumerate() {
            pt.rpm = (DYNO_MIN_RPM + i * DYNO_STEP_RPM) as f32;
        }

        let mut sim = Self {
            sample_rate: sample_rate.max(22050.0),
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
            rev_limiter_active: false,
            shift_cut_timer: 0.0,
            blip_timer: 0.0,
            shift_cooldown: 0.0,
            last_gear: 1,

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

            left_wg: BankWaveguide::new(sample_rate.max(22050.0)),
            right_wg: BankWaveguide::new(sample_rate.max(22050.0)),
            intake_buf: [0.0; 256],
            intake_ptr: 0,
            intake_lp: 0.0,
            dc_prev_in_l: 0.0,
            dc_prev_out_l: 0.0,
            dc_prev_in_r: 0.0,
            dc_prev_out_r: 0.0,
            air_lp_l: 0.0,
            air_lp_r: 0.0,
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

        for c in &mut self.cylinders {
            *c = Cylinder::new(c.firing_offset_rad);
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
            ctrl.bench_mode = BenchMode::DynoSweep;
            self.dyno_sweep_running = true;
            self.dyno_sweep_rpm = 5500.0;
            if self.omega_rad_s < 520.0 {
                self.omega_rad_s = 5500.0 * 2.0 * PI / 60.0;
            }
        }

        const SUBSTEPS: usize = 2;
        let dt = 1.0 / (self.sample_rate * SUBSTEPS as f64);

        let mut left_drive = [0.0f32; 5];
        let mut right_drive = [0.0f32; 5];
        let mut intake_drive = 0.0f32;

        for _ in 0..SUBSTEPS {
            self.step_physics_substep(dt, ctrl);
            for i in 0..5 {
                left_drive[i] += self.cylinders[i].ex_flow_pulse as f32 * 1.6;
                right_drive[i] += self.cylinders[i + 5].ex_flow_pulse as f32 * 1.6;
            }
            for c in &self.cylinders {
                intake_drive += c.in_flow_pulse as f32 * 1.2;
            }
        }

        // Procedural audio synthesis at sample rate
        let pop_n_l = self.fast_noise();
        let pop_n_r = self.fast_noise();
        let ex_l = self.left_wg.step(left_drive, pop_n_l);
        let ex_r = self.right_wg.step(right_drive, pop_n_r);

        // Variable intake trumpet waveguide resonator
        let total_intake_len = 0.085 + self.trumpet_length_m;
        let intake_delay = (((2.0 * total_intake_len / 346.0) * self.sample_rate).round() as usize)
            .clamp(8, 240);
        let ip = self.intake_ptr;
        let i_read = (ip + 256 - intake_delay) & 255;
        let i_del = self.intake_buf[i_read];
        let i_wave = (intake_drive * 0.65 - 0.55 * i_del).clamp(-4.0, 4.0);
        self.intake_buf[ip] = i_wave;
        self.intake_ptr = (ip + 1) & 255;
        self.intake_lp += 0.25 * (i_wave - self.intake_lp);
        let intake_snd = ((i_wave - self.intake_lp) * 0.6 + self.intake_lp * 0.4)
            * (0.18 + 0.82 * self.effective_throttle as f32);

        // Straight-cut camshaft gear & pneumatic valvetrain mechanical whine
        let rpm = (self.omega_rad_s * 60.0 / (2.0 * PI)).max(0.0);
        let mech_amp = ((rpm / 19000.0) as f32).clamp(0.0, 1.1) * 0.022;
        let ca = self.crank_angle_rad;
        let gear_whine = (((ca * 11.0).sin() * 0.5 + (ca * 22.0).sin() * 0.35 + (ca * 5.0).sin() * 0.25)
            as f32)
            * mech_amp;

        // External pneumatic starter gun turbine sound
        let starter_snd = if ctrl.starter_active {
            let st_freq = 420.0 + rpm * 0.35;
            self.starter_phase = (self.starter_phase + 2.0 * PI * st_freq / self.sample_rate) % (2.0 * PI);
            let air_hiss = self.fast_noise() * 0.04;
            (self.starter_phase.sin() as f32 * 0.08) + air_hiss
        } else {
            0.0
        };

        // Stereo mix (Left Bank skewed L, Right Bank skewed R, Intake & Mech centered)
        let raw_l = ex_l * 0.58 + ex_r * 0.20 + intake_snd * 0.26 + gear_whine + starter_snd;
        let raw_r = ex_r * 0.58 + ex_l * 0.20 + intake_snd * 0.26 + gear_whine + starter_snd;

        // 20 Hz DC blocking filter + mild air absorption smoothing
        let dc_l = raw_l - self.dc_prev_in_l + 0.996 * self.dc_prev_out_l;
        self.dc_prev_in_l = raw_l;
        self.dc_prev_out_l = dc_l;

        let dc_r = raw_r - self.dc_prev_in_r + 0.996 * self.dc_prev_out_r;
        self.dc_prev_in_r = raw_r;
        self.dc_prev_out_r = dc_r;

        self.air_lp_l += 0.62 * (dc_l - self.air_lp_l);
        self.air_lp_r += 0.62 * (dc_r - self.air_lp_r);

        let gain = if ctrl.muted { 0.0 } else { ctrl.master_volume * 0.72 };
        let out_l = (self.air_lp_l * gain).tanh();
        let out_r = (self.air_lp_r * gain).tanh();

        // Update oscilloscope trace (decimated 2:1 so waveform displays ~10ms window cleanly)
        self.scope_decim = (self.scope_decim + 1) & 1;
        if self.scope_decim == 0 {
            let sp = self.scope_ptr;
            self.scope_left[sp] = ex_l;
            self.scope_right[sp] = ex_r;
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

        // Handle gear changes (Upshift ignition cut / Downshift auto-blip)
        let in_gearbox_mode = matches!(
            ctrl.bench_mode,
            BenchMode::TrackManualGearbox | BenchMode::MonzaHotlapAuto
        );
        if in_gearbox_mode {
            let req_g = ctrl.requested_gear.clamp(1, 7);
            if req_g > self.last_gear {
                // Upshift ignition cut (25 ms) + RPM ratio drop
                self.shift_cut_timer = 0.025;
                self.shift_cooldown = 0.09;
                let ratio_scale = GEAR_RATIOS[req_g - 1] / GEAR_RATIOS[self.last_gear - 1];
                self.omega_rad_s *= ratio_scale;
                self.last_gear = req_g;
            } else if req_g < self.last_gear {
                // Downshift auto-blip (55 ms) + RPM ratio jump
                self.blip_timer = 0.055;
                self.shift_cooldown = 0.09;
                let ratio_scale = GEAR_RATIOS[req_g - 1] / GEAR_RATIOS[self.last_gear - 1];
                self.omega_rad_s = (self.omega_rad_s * ratio_scale).min(2100.0);
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

        // Dyno sweep override
        if ctrl.bench_mode == BenchMode::DynoSweep && self.dyno_sweep_running {
            requested_throttle = 1.0;
            self.dyno_sweep_rpm += 2200.0 * dt;
            if self.dyno_sweep_rpm >= ctrl.rev_limit_rpm.min(20000.0) {
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
        let global_spark_cut = self.rev_cut_timer > 0.0 || self.shift_cut_timer > 0.0;

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
        let mut pop_trigger_l = false;
        let mut pop_trigger_r = false;

        // Spark advance (deg BTDC): ramps from 14 deg at idle up to 28 deg MBT at 19k RPM + ECU offset
        let base_adv_deg = 14.0 + 14.0 * ((rpm - 4000.0) / 15000.0).clamp(0.0, 1.0);
        let spark_adv_deg = (base_adv_deg + ctrl.spark_offset_deg).clamp(2.0, 48.0);
        let spark_angle_rad = (720.0 - spark_adv_deg).to_radians();

        let p_ex_manifold = P_ATM + 3500.0 * (rpm / 19000.0).powi(2);
        let t_ex_manifold = (self.egt_c + 273.15).max(300.0);

        for idx in 0..NUM_CYL {
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

            let mut dm_in = if a_in > 1e-7 {
                orifice_mass_flow(p_port_in, T_ATM + 12.0, cyl.pressure_pa, cyl.temp_k, a_in) * dt
            } else {
                0.0
            };
            let mut dm_ex = if a_ex > 1e-7 {
                orifice_mass_flow(cyl.pressure_pa, cyl.temp_k, p_ex_manifold, t_ex_manifold, a_ex) * dt
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
            // Note: negative dm_ex (exhaust reversion during overlap) adds inert exhaust to air_mass, NOT fresh_air_mass!
            cyl.fresh_air_mass = cyl.fresh_air_mass.min(m_old_track + dm_in.max(0.0));

            dm_plenum_out += dm_in / dt;

            // Fuel injection during intake stroke (400..540 deg)
            if dm_in > 0.0 && ctrl.fuel_pump_on {
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

                    // Oxygen-limited chemical energy based strictly on FRESH trapped air (exhaust dilution doesn't burn!)
                    let max_burn_fuel = cyl.fresh_air_mass / AFR_STOICH;
                    let burn_fuel = cyl.fuel_mass.min(max_burn_fuel * 1.04);
                    let rich_bonus = if ctrl.lambda_target < 1.0 {
                        1.0 + 0.05 * (1.0 - ctrl.lambda_target)
                    } else {
                        1.0 - 0.35 * (ctrl.lambda_target - 1.0)
                    };
                    cyl.comb_energy_total = (burn_fuel * Q_LHV_FUEL * 0.85 * rich_bonus).max(0.0);
                    cyl.fuel_mass = (cyl.fuel_mass - burn_fuel).max(0.0);
                    cyl.fresh_air_mass = (cyl.fresh_air_mass - burn_fuel * AFR_STOICH).max(0.0);
                    cyl.unburnt_fuel = 0.0;

                    // Detonation / Knock condition: excessive spark advance, lean WOT, or overheating
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
                    // Ignition cut or unlit mixture sends unburnt fuel into hot headers -> crackle pops!
                    cyl.unburnt_fuel = cyl.fuel_mass;
                    cyl.fuel_mass = 0.0;
                    cyl.spark_cut_this_cycle = true;
                }
            }

            // Wiebe heat release (~66 deg burn duration -> MFB50 around ~7 deg ATDC)
            let mut dq_comb = 0.0;
            if cyl.combusting {
                let burn_dur = 1.15; // ~66 deg crank angle
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

            // 1st-Law Energy Balance: dU = dQ_comb - dQ_wall - P*dV + h_in*dm_in - h_out*dm_ex
            let m_old = cyl.air_mass.max(1e-6);
            let m_new = (m_old + dm_in - dm_ex).max(1e-6);

            let h_in_spec = GAMMA_AIR * CV_GAS * (if dm_in >= 0.0 { T_ATM + 12.0 } else { cyl.temp_k });
            let h_ex_spec = GAMMA_AIR * CV_GAS * (if dm_ex >= 0.0 { cyl.temp_k } else { t_ex_manifold });

            // Wall heat transfer (Woschni-like)
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

            // Exhaust valve opening blowdown & overrun pop detection
            if dm_ex > 0.0 {
                let p_ratio = ((cyl.pressure_pa - p_ex_manifold) / 100_000.0).max(0.0);
                cyl.ex_flow_pulse = (dm_ex / dt) * (1.0 + 0.55 * p_ratio.sqrt());
                if cyl.unburnt_fuel > 1e-6 && self.egt_c > 350.0 && theta > 2.2 {
                    cyl.unburnt_fuel = 0.0;
                    if idx < 5 {
                        pop_trigger_l = true;
                    } else {
                        pop_trigger_r = true;
                    }
                }
            } else {
                cyl.ex_flow_pulse = (dm_ex / dt) * 0.25;
            }
            cyl.in_flow_pulse = dm_in / dt;

            if cyl.spark_timer > 0.0 {
                cyl.spark_timer = (cyl.spark_timer - dt).max(0.0);
            }
            if cyl.knock_timer > 0.0 {
                cyl.knock_timer = (cyl.knock_timer - dt).max(0.0);
            }

            // Virtual-work crank gas torque
            total_gas_torque += (cyl.pressure_pa - P_ATM) * PISTON_AREA_M2 * dx_dtheta;
            sum_intake_mass += cyl.last_intake_mass;

            // Update Cylinder 1 P-V Diagram bucket
            if idx == 0 {
                let pv_idx = (((theta / FOUR_PI) * PV_POINTS as f64) as usize).min(PV_POINTS - 1);
                self.pv_trace[pv_idx] = ((vol * 1e6) as f32, (cyl.pressure_pa * 1e-5) as f32);
            }
        }

        if pop_trigger_l {
            self.left_wg.pop_env = (self.left_wg.pop_env + 0.85).min(1.8);
        }
        if pop_trigger_r {
            self.right_wg.pop_env = (self.right_wg.pop_env + 0.85).min(1.8);
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
                let target_omega = ctrl.dyno_hold_rpm * 2.0 * PI / 60.0;
                let err = self.omega_rad_s - target_omega;
                net_load_torque = (err * 45.0).clamp(-350.0, 800.0);
            }
            BenchMode::DynoSweep => {
                if self.dyno_sweep_running {
                    let target_omega = self.dyno_sweep_rpm * 2.0 * PI / 60.0;
                    let err = self.omega_rad_s - target_omega;
                    net_load_torque = (err * 12.0).clamp(-350.0, 700.0);
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
                    self.car_speed_ms = (self.omega_rad_s * wheel_to_crank).max(0.0);
                }
                self.monza_dist_m = (self.monza_dist_m + self.car_speed_ms * dt) % 5793.0;
            }
        }

        let net_torque = brake_engine_torque + starter_torque - net_load_torque;
        self.omega_rad_s = (self.omega_rad_s + (net_torque / eff_inertia) * dt).clamp(0.0, 2350.0);
        rpm = self.omega_rad_s * 60.0 / (2.0 * PI);

        // 10. Thermal & Exhaust Gas Temperature (EGT) Simulation
        let heater_coolant = if ctrl.coolant_heater_on { 5.2 } else { 0.0 };
        let heater_oil = if ctrl.oil_heater_on { 5.0 } else { 0.0 };
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

        // 11. Filtered Telemetry & Dyno Curve Recording
        let filter_alpha = (18.0 * dt).min(1.0);
        self.filtered_torque_nm += (brake_engine_torque - self.filtered_torque_nm) * filter_alpha;
        let inst_bhp = (self.filtered_torque_nm * self.omega_rad_s) / 745.69987;
        self.filtered_power_bhp += (inst_bhp - self.filtered_power_bhp) * filter_alpha;

        // Volumetric Efficiency across 10 cylinders
        let theoretical_air = 1.184 * DISP_VOL_M3 * NUM_CYL as f64;
        let ve = (sum_intake_mass / theoretical_air) * 100.0;
        self.filtered_ve += (ve - self.filtered_ve) * filter_alpha;

        if rpm >= DYNO_MIN_RPM as f64 && rpm <= DYNO_MAX_RPM as f64 && self.effective_throttle > 0.85 {
            let bin = (((rpm - DYNO_MIN_RPM as f64) / DYNO_STEP_RPM as f64).round() as usize)
                .min(DYNO_BINS - 1);
            let t_nm = self.filtered_torque_nm.max(0.0) as f32;
            let p_bhp = self.filtered_power_bhp.max(0.0) as f32;
            if !self.dyno_curve[bin].recorded || p_bhp > self.dyno_curve[bin].power_bhp {
                self.dyno_curve[bin].torque_nm = t_nm;
                self.dyno_curve[bin].power_bhp = p_bhp;
                self.dyno_curve[bin].recorded = true;
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
}
