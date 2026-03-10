# FMCW Radar GPU Pipeline (Multi-Target Tracking)

## 1. Overview

This project implements a CUDA-based FMCW radar processing pipeline for offline replay and online integration.

Current end-to-end flow:

1. Ingest multi-channel IQ payload.
2. GPU unpack + slow-time window.
3. Doppler FFT per channel.
4. Sum-channel RD power map.
5. CFAR + NMS peak extraction on RD map.
6. Monopulse angle/range/velocity estimation for detections.
7. Host-side post processing:
   - basic threshold filtering
   - DBSCAN candidate merging
   - optional Doppler-line clutter suppression
8. Multi-target tracking (Kalman + gated data association + track lifecycle).
9. Track output to CSV.

Executable: `radar_app`

## 2. Repository Layout

```text
app/                    # Main executable entry and replay loop
gpu/
  RadarPipeline.cu      # Main GPU + host post-processing pipeline
  kernels/              # unpack / power / cfar / monopulse kernels
tracking/
  MultiTargetTracker.*  # Multi-target tracking logic
io/                     # OfflineReplay, UDP receiver, frame assembly
model/                  # RadarConfig, target structs, channel map
runtime/                # Logger, metrics
matlab/
  plot_multi_track.m    # Track visualization script
CMakeLists.txt
```

## 3. Build

```bash
rm -rf build
/usr/bin/cmake -S . -B build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=86
/usr/bin/cmake --build build -j$(nproc)
```

## 4. Run Offline Replay

`radar_app` reads these files from current working directory:

- `output_he.dat` (sum channel)
- `output_cha1.dat` (azimuth diff channel)
- `output_cha2.dat` (elevation diff channel)

### 4.1 Profiles

Two runtime tuning profiles are supported:

- `single` (default): conservative, stable for single-target-like data.
- `multi`: more permissive for multi-target scenes.

```bash
cd build
./radar_app --profile single
./radar_app --profile multi
```

Help:

```bash
./radar_app --help
```

Console output includes:

- `[CFAR] hits=... peaks=...`
- `[DBSCAN] in=... clusters=... out=...`
- `[DOPPLER] in=... suppressed_bins=... out=...` (when enabled)
- `[Offline Frame k] detections=n tracks=m`
- `[Det] ...` and `[Track] ...`

## 5. Output Files

### 5.1 Multi-track CSV

Offline run writes:

- `offline_multi_track.csv`

Fields:

- `frame`
- `track_id` (`-1` means no output track in this frame)
- `confirmed` (`1` confirmed, `0` otherwise)
- `age` (track lifetime in frames)
- `hits` (associated detection count)
- `missed` (consecutive misses)
- `rbin`
- `dbin_c` (centered Doppler bin)
- `dbin_u` (unshifted Doppler bin)
- `range_m`
- `vel_mps`
- `snr_db`
- `az_deg`
- `el_deg`
- `power`

### 5.2 RD Power Dump (debug)

`power_map_frame_XXX.bin` may be generated for RD comparison/debug.

## 6. Tracking Semantics

Track line example:

```text
[Track] id=98 conf=1 age=18 hits=18 miss=0 rbin=24 dbin_c=-15 range=3.6 m vel=-3.0 m/s snr=27.54 dB
```

Meaning:

- `id`: track identity.
- `conf`: whether the track is confirmed by hit history.
- `age`: number of frames since track birth.
- `hits`: number of successful updates.
- `miss`: consecutive unmatched frames.
- `range/vel`: filtered track state.

## 7. Key Parameters

Parameters live in `model/RadarConfig.h`, and profile defaults are set in `app/main.cpp`.

### 7.1 Detection / Post-processing

- `cfar_peak_min_snr_db`
- `cfar_peak_half_window`
- `post_min_snr_db`
- `post_top_k`
- `dbscan_*`
- `post_doppler_line_suppress_enable`
- `post_doppler_line_min_points`
- `post_doppler_line_keep_per_bin`

### 7.2 Tracking

- `tracking_gate_range_m`
- `tracking_gate_velocity_mps`
- `tracking_confirm_hits`
- `tracking_max_missed_frames`
- `tracking_spawn_min_snr_db`
- `tracking_spawn_exclusion_range_m`
- `tracking_spawn_exclusion_velocity_mps`
- `tracking_output_only_updated_tracks`
- `tracking_output_min_age`
- `tracking_output_min_hits`

For suppressing short-lived false tracks, increase:

- `tracking_output_min_age`
- `tracking_output_min_hits`

## 8. MATLAB Visualization

Script:

- `matlab/plot_multi_track.m`

Usage:

```matlab
cd('/home/whj/cuda_workplace/radar_app/matlab');
plot_multi_track;
```

The script:

- loads `offline_multi_track.csv`
- keeps confirmed tracks
- filters short tracks by `min_track_rows` (default `6`)
- plots:
  - frame-range
  - frame-velocity
  - frame-SNR
  - range-velocity phase plot
  - bin-domain overview (`rbin`, `dbin_c`)

## 9. Troubleshooting

### 9.1 `CUDA error: OS call failed or operation not supported on this OS`

GPU runtime is not available in the environment (common in misconfigured WSL/container). Verify:

```bash
nvidia-smi
```

### 9.2 Too many detections/tracks

Try:

- increasing `cfar_peak_min_snr_db`
- increasing `post_min_snr_db`
- increasing `dbscan_min_points`
- enabling/tuning Doppler-line suppression
- increasing `tracking_output_min_hits` / `tracking_output_min_age`

### 9.3 Too many track breaks

Try:

- reducing `tracking_confirm_hits`
- increasing `tracking_max_missed_frames`
- slightly relaxing `tracking_gate_*`

## 10. Git Workflow

Current multi-target work is on branch:

- `feature/multi-target-tracking`

Typical commands:

```bash
git status
git add .
git commit -m "feat: multi-target tracking pipeline and tuning profiles"
git push -u origin feature/multi-target-tracking
```

