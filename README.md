# FMCW Radar GPU Pipeline (`radar_app`)

## 1. Project Overview

This project implements an offline/online FMCW radar processing pipeline on CUDA.

Current workflow (aligned with your S1-S6 description):

1. High-speed data is streamed to host.
2. CPU does packet/frame assembly and ping-pong style buffering.
3. GPU unpacks multi-channel IQ and converts to complex format.
4. GPU performs RD-related processing, CFAR, and candidate extraction.
5. GPU/CPU post-processes candidates (DBSCAN + optional single-track selection).
6. Target list is sent back to CPU for display/logging.

Main executable: `radar_app`

## 2. Repository Layout

```text
app/                    # entry + offline replay loop
gpu/
  RadarPipeline.cu      # pipeline orchestration
  kernels/              # unpack/power/cfar/monopulse kernels
io/                     # offline replay, UDP receiver, frame assembler
model/                  # RadarConfig, target types, channel map
runtime/                # logger/metrics
CMakeLists.txt
```

## 3. Prerequisites

- Linux/WSL2 with NVIDIA GPU support
- NVIDIA driver installed
- CUDA toolkit available at `/usr/local/cuda`
- `cmake >= 3.16`
- `g++` with C++17 support

Recommended checks:

```bash
which nvcc
nvcc --version
/usr/bin/cmake --version
nvidia-smi
```

Important: `which nvcc` should point to a valid CUDA toolkit (for example `/usr/local/cuda/bin/nvcc`), not an old/incompatible toolchain.

## 4. Build

From repository root:

```bash
rm -rf build
/usr/bin/cmake -S . -B build \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=86
/usr/bin/cmake --build build -j$(nproc)
```

Default arch in `CMakeLists.txt` is currently `86` (RTX 30 series style setup).

## 5. Offline Replay Run

`app/main.cpp` currently expects these files in **current working directory**:

- `output_he.dat` (sum channel)
- `output_cha1.dat` (az diff)
- `output_cha2.dat` (el diff)

Typical usage:

```bash
cd build
./radar_app
```

Console prints include:

- `[CFAR] hits=... peaks=...`
- `[DBSCAN] in=... clusters=... out=...`
- `[Offline Frame k] targets=n`
- `[Primary] ...` (selected primary target)

## 6. Output Files

### 6.1 Primary Track CSV

Offline run writes:

- `offline_primary_track.csv`

Columns:

- `frame`
- `valid` (`1`: primary exists, `0`: none)
- `targets` (target count in this frame)
- `primary_idx` (index in current frame target list)
- `rbin`
- `dbin_c` (centered Doppler bin)
- `dbin_u` (unshifted Doppler bin)
- `range_m`
- `vel_mps`
- `snr_db`
- `az_deg`
- `el_deg`

### 6.2 Optional RD Power Dumps

Some runs may produce `power_map_frame_XXX.bin` for debug visualization.

## 7. Key Runtime Tuning Parameters

Defined in `model/RadarConfig.h`, overridden in `app/main.cpp`.

### CFAR/NMS

| Parameter | Effect if increased | Effect if decreased |
|---|---|---|
| `cfar_peak_half_window` | stronger suppression, fewer nearby peaks | more peaks, easier multi-peak clutter |
| `cfar_peak_min_snr_db` | fewer false alarms | more detections, more false alarms |
| `cfar_zero_doppler_suppress_bins` | suppress near-zero Doppler clutter | keep slow/zero-velocity targets |
| `cfar_near_range_suppress_bins` | remove near-range artifacts | keep close targets |

### Post-filter

| Parameter | Effect |
|---|---|
| `post_min_snr_db` | remove weak detections before tracking |
| `post_max_abs_vel_mps` | reject impossible high-speed candidates |
| `post_top_k` | keep only strongest K targets (`0` = keep all) |

### DBSCAN (range-doppler)

| Parameter | Effect if increased | Effect if decreased |
|---|---|---|
| `dbscan_eps_range_m` | easier merging across range bins | stricter cluster split |
| `dbscan_eps_velocity_mps` | easier merging across Doppler bins | stricter velocity separation |
| `dbscan_min_points` | fewer tiny clusters/noise accepted | more tiny clusters accepted |
| `dbscan_keep_noise` | isolated points may survive | isolated points removed |

### Single-target temporal mode

| Parameter | Effect |
|---|---|
| `single_target_mode` | continuity-aware target selection |
| `single_track_gate_range_m` | allowed inter-frame range jump |
| `single_track_gate_velocity_mps` | allowed inter-frame velocity jump |
| `single_track_snr_near_max_db` | only evaluate points near top SNR |

## 8. MATLAB Validation Example

Use this to inspect extracted primary trajectory:

```matlab
clc; clear; close all;

T = readtable('offline_primary_track.csv');

% valid primary only
Tv = T(T.valid == 1, :);

figure('Name','Primary Track');
subplot(2,1,1);
plot(Tv.frame, Tv.rbin, 'o-'); grid on;
xlabel('frame'); ylabel('rbin');
title('Range Bin Trajectory');

subplot(2,1,2);
plot(Tv.frame, Tv.dbin_c, 'o-'); grid on;
xlabel('frame'); ylabel('dbin\_c');
title('Centered Doppler Bin Trajectory');
```

Interpretation guideline:

- Long plateau at a fixed `dbin_c` is normal quantization behavior (bin-level velocity).
- Sign switch (`+` to `-`) usually indicates radial direction change.
- A few outlier frames at start/end are common; tune temporal gates to stabilize.

## 9. Common Issues

### `ptxas fatal: Value 'sm_30' is not defined`

Cause: old/incompatible CUDA toolchain being picked up.

Fix:

- Use correct cmake + nvcc explicitly.
- Reconfigure with `-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`.

### CMake uses wrong binary (for example from another toolkit)

Fix:

```bash
which cmake
/usr/bin/cmake --version
```

Then call `/usr/bin/cmake` explicitly.

### Link error `undefined reference to launch_cfar_and_peak_extract...`

Cause: declaration/definition signature mismatch between pipeline and kernel launcher.

Fix: keep the launcher function signature consistent in both declaration and implementation.

### Too many targets / unstable first frames

Try:

- increase `cfar_peak_min_snr_db`
- increase `cfar_peak_half_window`
- set `dbscan_min_points = 2~3`
- keep `single_target_mode = true` for single-target datasets

### Runtime error `CUDA error: OS call failed or operation not supported on this OS`

Often environment related (no GPU access in current runtime/WSL session).
Verify GPU availability with `nvidia-smi` and CUDA runtime access.

## 10. Git Quick Start

If local commits already exist:

```bash
git remote add origin <repo-url>
git push -u origin main
```

Tag a stable checkpoint:

```bash
git tag -a v0.1.0 -m "first working offline pipeline"
git push origin v0.1.0
```
