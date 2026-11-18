mod sim;
mod ui;

use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{SampleFormat, Stream, StreamConfig};
use eframe::egui;

use sim::{BenchMode, EngineSim, SharedControls, TelemetrySnapshot};
use ui::FormulaApp;

const SIM_SAMPLE_RATE: f64 = 48_000.0;
const RING_CAP: usize = 8192;
const RING_MASK: usize = RING_CAP - 1;
const TARGET_LEAD_FRAMES: usize = 2048; // ~42 ms headroom for Windows 15.6 ms sleep granularity + USB jitter

fn main() -> eframe::Result<()> {
    let controls = Arc::new(Mutex::new(SharedControls::default()));
    let telemetry = Arc::new(Mutex::new(TelemetrySnapshot::default()));

    // Keep the audio stream & lock-free physics producer thread alive for the lifetime of main()
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

#[inline]
fn pack_stereo(l: f32, r: f32) -> u64 {
    (l.to_bits() as u64) | ((r.to_bits() as u64) << 32)
}

#[inline]
fn unpack_stereo(bits: u64) -> (f32, f32) {
    let l = f32::from_bits(bits as u32);
    let r = f32::from_bits((bits >> 32) as u32);
    (l, r)
}

/// Lock-free single-producer / single-consumer stereo audio ring buffer.
struct AudioRingBuffer {
    slots: Box<[AtomicU64]>,
    write_idx: AtomicUsize,
    read_idx: AtomicUsize,
}

impl AudioRingBuffer {
    fn new() -> Self {
        let slots = (0..RING_CAP)
            .map(|_| AtomicU64::new(0))
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Self {
            slots,
            write_idx: AtomicUsize::new(0),
            read_idx: AtomicUsize::new(0),
        }
    }

    #[inline]
    fn queued(&self) -> usize {
        let w = self.write_idx.load(Ordering::Acquire);
        let r = self.read_idx.load(Ordering::Acquire);
        w.wrapping_sub(r)
    }

    #[inline]
    fn push_frame(&self, l: f32, r: f32) -> bool {
        let wr = self.write_idx.load(Ordering::Relaxed);
        let rd = self.read_idx.load(Ordering::Acquire);
        if wr.wrapping_sub(rd) >= RING_CAP - 1 {
            return false;
        }
        self.slots[wr & RING_MASK].store(pack_stereo(l, r), Ordering::Relaxed);
        self.write_idx.store(wr.wrapping_add(1), Ordering::Release);
        true
    }

    #[inline]
    fn pop_frame(&self) -> Option<(f32, f32)> {
        let r = self.read_idx.load(Ordering::Relaxed);
        let w = self.write_idx.load(Ordering::Acquire);
        if r == w {
            return None;
        }
        let bits = self.slots[r & RING_MASK].load(Ordering::Relaxed);
        self.read_idx.store(r.wrapping_add(1), Ordering::Release);
        Some(unpack_stereo(bits))
    }
}

/// Real-time resampler state owned exclusively by the cpal output callback.
struct ResampleReader {
    ring: Arc<AudioRingBuffer>,
    step: f64,
    phase: f64,
    cur: (f32, f32),
    next: (f32, f32),
}

impl ResampleReader {
    fn new(ring: Arc<AudioRingBuffer>, device_sample_rate: f64) -> Self {
        Self {
            ring,
            step: SIM_SAMPLE_RATE / device_sample_rate.max(8000.0),
            phase: 0.0,
            cur: (0.0, 0.0),
            next: (0.0, 0.0),
        }
    }

    #[inline]
    fn next_stereo(&mut self) -> (f32, f32) {
        if (self.step - 1.0).abs() < 1e-4 {
            if let Some(frame) = self.ring.pop_frame() {
                self.cur = frame;
                self.next = frame;
            } else {
                self.cur = (self.cur.0 * 0.992, self.cur.1 * 0.992);
                self.next = self.cur;
            }
            return self.cur;
        }

        let t = self.phase as f32;
        let out_l = self.cur.0 + (self.next.0 - self.cur.0) * t;
        let out_r = self.cur.1 + (self.next.1 - self.cur.1) * t;

        self.phase += self.step;
        while self.phase >= 1.0 {
            self.phase -= 1.0;
            self.cur = self.next;
            if let Some(frame) = self.ring.pop_frame() {
                self.next = frame;
            } else {
                // Smooth exponential tail if briefly empty during device reset/startup
                self.next = (self.next.0 * 0.992, self.next.1 * 0.992);
            }
        }
        (out_l, out_r)
    }
}

struct AudioHandle {
    #[allow(dead_code)]
    stream: Option<Stream>,
    stop: Arc<AtomicBool>,
    producer: Option<JoinHandle<()>>,
}

impl Drop for AudioHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.producer.take() {
            let _ = handle.join();
        }
    }
}

fn start_audio_or_fallback(
    controls: Arc<Mutex<SharedControls>>,
    telemetry: Arc<Mutex<TelemetrySnapshot>>,
) -> AudioHandle {
    let ring = Arc::new(AudioRingBuffer::new());
    let stop = Arc::new(AtomicBool::new(false));
    let mut sim = EngineSim::new(SIM_SAMPLE_RATE);
    let mut local_ctrl = SharedControls::default();

    // Warm-fill initial buffer headroom before starting the WASAPI stream
    for _ in 0..768 {
        let (l, r) = sim.step_sample(&mut local_ctrl);
        let _ = ring.push_frame(l, r);
    }

    let stream = match try_build_audio_stream(Arc::clone(&ring)) {
        Ok(s) if s.play().is_ok() => Some(s),
        _ => None,
    };
    let has_audio = stream.is_some();

    let ring_prod = Arc::clone(&ring);
    let stop_prod = Arc::clone(&stop);
    let producer = std::thread::spawn(move || {
        let mut snap_counter = 0usize;
        let snap_interval = (SIM_SAMPLE_RATE / 75.0).round() as usize;
        let mut last = Instant::now();

        while !stop_prod.load(Ordering::Relaxed) {
            if has_audio {
                let queued = ring_prod.queued();
                if queued < TARGET_LEAD_FRAMES {
                    sync_controls_in(&controls, &mut local_ctrl);
                    let prev_mode = local_ctrl.bench_mode;
                    let batch = (TARGET_LEAD_FRAMES - queued).min(256);
                    for _ in 0..batch {
                        let (l, r) = sim.step_sample(&mut local_ctrl);
                        if !ring_prod.push_frame(l, r) {
                            break;
                        }
                        snap_counter += 1;
                    }
                    sync_controls_out(&controls, &local_ctrl, prev_mode);

                    if snap_counter >= snap_interval {
                        snap_counter = 0;
                        if let Ok(mut t) = telemetry.try_lock() {
                            *t = sim.snapshot(&local_ctrl);
                        }
                    }
                } else {
                    std::thread::sleep(Duration::from_millis(1));
                }
            } else {
                // Fallback software physics loop if no audio device is available
                std::thread::sleep(Duration::from_millis(8));
                let now = Instant::now();
                let elapsed = (now - last).as_secs_f64().min(0.03);
                last = now;

                sync_controls_in(&controls, &mut local_ctrl);
                let prev_mode = local_ctrl.bench_mode;
                let samples = (elapsed * SIM_SAMPLE_RATE).round() as usize;
                for _ in 0..samples {
                    let _ = sim.step_sample(&mut local_ctrl);
                }
                sync_controls_out(&controls, &local_ctrl, prev_mode);
                if let Ok(mut t) = telemetry.try_lock() {
                    *t = sim.snapshot(&local_ctrl);
                }
            }
        }
    });

    AudioHandle {
        stream,
        stop,
        producer: Some(producer),
    }
}

fn sync_controls_in(controls: &Arc<Mutex<SharedControls>>, local: &mut SharedControls) {
    if let Ok(mut guard) = controls.try_lock() {
        *local = *guard;
        guard.quick_start_trigger = false;
        guard.reset_repair_trigger = false;
        guard.start_dyno_sweep = false;
        guard.clear_dyno_trace = false;
        guard.manual_crank_delta_rad = 0.0;
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

fn try_build_audio_stream(ring: Arc<AudioRingBuffer>) -> Result<Stream, String> {
    let host = cpal::default_host();
    let device = host
        .default_output_device()
        .ok_or_else(|| "No output audio device found".to_string())?;
    let supported = device
        .default_output_config()
        .map_err(|e| format!("Failed output config: {e}"))?;

    let device_sr = supported.sample_rate().0 as f64;
    let channels = supported.channels() as usize;
    let format = supported.sample_format();
    let config: StreamConfig = supported.into();

    let mut reader = ResampleReader::new(ring, device_sr);
    let err_fn = |err| eprintln!("Audio stream error: {err}");

    let stream = match format {
        SampleFormat::F32 => device.build_output_stream(
            &config,
            move |data: &mut [f32], _| {
                process_buffer(data, channels, &mut reader, |s| s);
            },
            err_fn,
            None,
        ),
        SampleFormat::I16 => device.build_output_stream(
            &config,
            move |data: &mut [i16], _| {
                process_buffer(data, channels, &mut reader, |s| {
                    (s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16
                });
            },
            err_fn,
            None,
        ),
        SampleFormat::U16 => device.build_output_stream(
            &config,
            move |data: &mut [u16], _| {
                process_buffer(data, channels, &mut reader, |s| {
                    ((s.clamp(-1.0, 1.0) * 0.5 + 0.5) * u16::MAX as f32) as u16
                });
            },
            err_fn,
            None,
        ),
        other => return Err(format!("Unsupported sample format: {other:?}")),
    }
    .map_err(|e| format!("Failed building stream: {e}"))?;

    Ok(stream)
}

#[inline]
fn process_buffer<T: Copy>(
    data: &mut [T],
    channels: usize,
    reader: &mut ResampleReader,
    convert: impl Fn(f32) -> T,
) {
    let ch = channels.max(1);
    for frame in data.chunks_mut(ch) {
        let (l, r) = reader.next_stereo();
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
    }
}
