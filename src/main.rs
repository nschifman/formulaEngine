mod sim;
mod ui;

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{SampleFormat, Stream, StreamConfig};
use eframe::egui;

use sim::{BenchMode, EngineSim, SharedControls, TelemetrySnapshot};
use ui::FormulaApp;

fn main() -> eframe::Result<()> {
    let controls = Arc::new(Mutex::new(SharedControls::default()));
    let telemetry = Arc::new(Mutex::new(TelemetrySnapshot::default()));

    // Keep the audio stream (or fallback physics thread) alive for the lifetime of main()
    let _audio_handle = start_audio_or_fallback(Arc::clone(&controls), Arc::clone(&telemetry));

    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1440.0, 880.0])
            .with_min_inner_size([1150.0, 700.0])
            .with_title("Formula Engine Simulator — 3.0L V10 Test Cell"),
        ..Default::default()
    };

    eframe::run_native(
        "Formula Engine Simulator",
        native_options,
        Box::new(|cc| {
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            Ok(Box::new(FormulaApp::new(controls, telemetry)))
        }),
    )
}

enum AudioHandle {
    Stream(#[allow(dead_code)] Stream),
    FallbackThread,
}

fn start_audio_or_fallback(
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
) -> AudioHandle {
    if let Ok(stream) = try_build_audio_stream(Arc::clone(&controls), Arc::clone(&telemetry)) {
        if stream.play().is_ok() {
            return AudioHandle::Stream(stream);
        }
    }

    // Fallback software physics loop if no audio device is available
    std::thread::spawn(move || {
        let mut sim = EngineSim::new(48_000.0);
        let mut local_ctrl = SharedControls::default();
        let mut last = Instant::now();
        loop {
            std::thread::sleep(Duration::from_millis(8));
            let now = Instant::now();
            let elapsed = (now - last).as_secs_f64().min(0.03);
            last = now;

            sync_controls_in(&controls, &mut local_ctrl);
            let prev_mode = local_ctrl.bench_mode;
            let samples = (elapsed * 48_000.0).round() as usize;
            for _ in 0..samples {
                let _ = sim.step_sample(&mut local_ctrl);
            }
            sync_controls_out(&controls, &local_ctrl, prev_mode);
            if let Ok(mut t) = telemetry.try_lock() {
                *t = sim.snapshot(&local_ctrl);
            }
        }
    });

    AudioHandle::FallbackThread
}

fn sync_controls_in(controls: &Arc<Mutex<SharedControls>>, local: &mut SharedControls) {
    if let Ok(mut guard) = controls.try_lock() {
        *local = *guard;
        guard.quick_start_trigger = false;
        guard.reset_repair_trigger = false;
        guard.start_dyno_sweep = false;
        guard.clear_dyno_trace = false;
    }
}

fn sync_controls_out(
    controls: &Arc<Mutex<SharedControls>>,
    local: &SharedControls,
    prev_mode: BenchMode,
) {
    if let Ok(mut guard) = controls.try_lock() {
        if prev_mode == BenchMode::DynoSweep && local.bench_mode != BenchMode::DynoSweep {
            guard.bench_mode = local.bench_mode;
            guard.dyno_brake_load = local.dyno_brake_load;
        }
        if local.bench_mode == BenchMode::MonzaHotlapAuto {
            guard.requested_gear = local.requested_gear;
        }
    }
}

fn try_build_audio_stream(
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
) -> Result<Stream, String> {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .ok_or_else(|| "No output audio device found".to_string())?;
    let supported = device
        .default_output_config()
        .map_err(|e| format!("Failed output config: {e}"))?;

    let sample_rate = supported.sample_rate().0 as f64;
    let channels = supported.channels() as usize;
    let format = supported.sample_format();
    let config: StreamConfig = supported.into();

    let mut sim = EngineSim::new(sample_rate);
    let mut local_ctrl = SharedControls::default();
    let mut snap_counter = 0usize;
    let snap_interval = (sample_rate / 75.0).round() as usize;

    let err_fn = |err| eprintln!("Audio stream error: {err}");

    let stream = match format {
        SampleFormat::F32 => device.build_output_stream(
            &config,
            move |data: &mut [f32], _| {
                process_buffer(
                    data,
                    channels,
                    &mut sim,
                    &mut local_ctrl,
                    &controls,
                    &telemetry,
                    &mut snap_counter,
                    snap_interval,
                    |s| s,
                );
            },
            err_fn,
            None,
        ),
        SampleFormat::I16 => device.build_output_stream(
            &config,
            move |data: &mut [i16], _| {
                process_buffer(
                    data,
                    channels,
                    &mut sim,
                    &mut local_ctrl,
                    &controls,
                    &telemetry,
                    &mut snap_counter,
                    snap_interval,
                    |s| (s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16,
                );
            },
            err_fn,
            None,
        ),
        SampleFormat::U16 => device.build_output_stream(
            &config,
            move |data: &mut [u16], _| {
                process_buffer(
                    data,
                    channels,
                    &mut sim,
                    &mut local_ctrl,
                    &controls,
                    &telemetry,
                    &mut snap_counter,
                    snap_interval,
                    |s| ((s.clamp(-1.0, 1.0) * 0.5 + 0.5) * u16::MAX as f32) as u16,
                );
            },
            err_fn,
            None,
        ),
        other => return Err(format!("Unsupported sample format: {other:?}")),
    }
    .map_err(|e| format!("Failed building stream: {e}"))?;

    Ok(stream)
}

#[allow(clippy::too_many_arguments)]
fn process_buffer<T: Copy>(
    data: &mut [T],
    channels: usize,
    sim: &mut EngineSim,
    local_ctrl: &mut SharedControls,
    controls: &Arc<Mutex<SharedControls>>,
    telemetry: &Arc<Mutex<TelemetrySnapshot>>,
    snap_counter: &mut usize,
    snap_interval: usize,
    convert: impl Fn(f32) -> T,
) {
    sync_controls_in(controls, local_ctrl);
    let prev_mode = local_ctrl.bench_mode;
    let ch = channels.max(1);

    for frame in data.chunks_mut(ch) {
        let (l, r) = sim.step_sample(local_ctrl);
        if ch == 1 {
            frame[0] = convert(0.5 * (l + r));
        } else {
            frame[0] = convert(l);
            frame[1] = convert(r);
            let mono = 0.5 * (l + r);
            for extra in &mut frame[2..] {
                *extra = convert(mono);
            }
        }
        *snap_counter += 1;
    }

    sync_controls_out(controls, local_ctrl, prev_mode);

    if *snap_counter >= snap_interval {
        *snap_counter = 0;
        if let Ok(mut t) = telemetry.try_lock() {
            *t = sim.snapshot(local_ctrl);
        }
    }
}
