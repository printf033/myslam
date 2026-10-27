# myslam

English | [简体中文](README_zh.md)

Visual-inertial odometry (VIO) in C++26. The backend uses GTSAM's
`IncrementalFixedLagSmoother` to solve IMU preintegration factors and visual reprojection factors
in a single factor graph, marginalizing states that leave the time window.

- **Language standard**: C++26 (enforced by CMake)
- **Dependencies**: [GTSAM](https://github.com/borglab/gtsam) 4.3, [mylog](https://github.com/printf033/mylog) (both submodules)
- **Optional**: LuaJIT 2.1 (configuration), OpenCV 4 (dataset replay tools)
- **Version**: single source is `project(VERSION)` in `CMakeLists.txt`; read at runtime via `myslam::kVersion`

---

## Contents

- [Architecture](#architecture)
- [Data flow](#data-flow)
- [Module responsibilities](#module-responsibilities)
- [Execution flow](#execution-flow)
- [Repository layout](#repository-layout)
- [Requirements](#requirements)
- [Building](#building)
- [Running](#running)
- [Configuration](#configuration)
- [Dataset replay](#dataset-replay)
- [Scripts](#scripts)
- [Tests](#tests)
- [Engineering conventions](#engineering-conventions)
- [Current limitations](#current-limitations)

---

## Architecture

The system has five parts, listed in call order:

1. **Initializer** `VioInitializer` — state machine `UNINITIALIZED → STATIONARY_COLLECTING →
   GRAVITY_ALIGNED → TRACKING_OK`, with `LOST_DEGRADED` on anomaly. Stationarity is decided from
   both accelerometer and gyro standard deviation; gyro bias calibration runs after gravity
   alignment, producing the initial pose, velocity, and bias.
2. **Keyframe decision** `KeyframeSelector` — decides whether the current camera frame becomes a
   keyframe, using average parallax, feature retention, and a time-based fallback. The fallback
   additionally requires a minimum triangulation parallax.
3. **Backend** `VioBackend` — combines `CombinedImuFactor` (IMU preintegration) with
   `ProjectionFactor` (visual reprojection) and solves through `IncrementalFixedLagSmoother` over
   the `slidingWindowLag` window, marginalizing states that leave it. Visual residuals use a Huber
   kernel.
4. **Propagator** `HighRatePropagator` — propagates state between optimizations at the IMU rate and
   resets its base to the optimized state after each optimization.
5. **Health monitor** `HealthMonitor` — checks bias magnitude, speed magnitude, and active
   observation count, returning alert text when a threshold is exceeded.

The simulation and recorded-dataset paths reuse all of the above; only the sensor source changes.

## Data flow

![Architecture](docs/architecture.svg)

The two columns are the two sample rates. Each column is split into an initialization stage and a
tracking stage; within a column the stages are selected by the current `SystemState` and are
mutually exclusive. Blue boxes are the same `VioBackend` instance invoked on both rate paths.

The main loop is driven by `VioSystem::run()`: time advances by the IMU step, each IMU sample
updates preintegration or propagation, camera frames trigger initialization or keyframe decisions,
and a keyframe triggers backend optimization followed by a propagator reset.

## Module responsibilities

| Module | Responsibility |
| :--- | :--- |
| `vio/params.hpp` | Sensor structs (`ImuMeasurement` / `CameraFrame` / `FeatureObservation`) and `VioConfig` |
| `vio/initializer.hpp` | `SystemState` state machine, stationarity detection, gravity alignment, bias calibration |
| `vio/keyframe_selector.hpp` | Keyframe decision returning `DecisionResult` (flag, reason, parallax, retention) |
| `vio/backend.hpp` | Factor graph construction, triangulation, layered outlier gating, marginalization, `latestNavState()` |
| `vio/propagator.hpp` | IMU-rate state propagation and base reset |
| `vio/health_monitor.hpp` | Bias, speed, and observation-count checks returning `HealthReport` |
| `vio/simulator.hpp` | 3D trajectory and IMU / camera observation generation with analytic ground truth |
| `vio/system.hpp` | Dual-rate main loop orchestration, `SystemConfig`, accuracy evaluation |
| `vio/vio.hpp` | Umbrella header including all of the above |
| `lua_config.hpp` | Lua configuration loading producing `ProjectConfig` |
| `log_config.hpp` | Thin wrapper over the mylog C++ API (level + file output) |
| `dataset/types.hpp` | Replay configuration and statistics structs; standard library only, no OpenCV |
| `dataset/subt_dataset.hpp` | SubT sequence reader: IMU CSV, camera timestamps, image index |
| `dataset/mei_tracker.hpp` | MEI model distortion correction and GFTT + LK optical-flow tracking |
| `dataset/dataset_runner.hpp` | Recorded-sequence replay orchestration reusing the VIO components |
| `dataset/visualizer.hpp` | OpenCV window and video output |
| `dataset/ply_export.hpp` | Export landmarks and trajectory as PLY |

`vio/*` and `dataset/types.hpp` include no OpenCV headers, so the project still builds and parses
its configuration when the dataset tools are disabled.

## Execution flow

### Initialization

`VioInitializer::feedImu()` accumulates stationary samples and tracks accelerometer and gyro
standard deviation. Once the samples qualify, `tryGravityAlignment()` derives roll/pitch from the
mean specific force, estimates bias from the mean gyro reading, and returns a `Rot3` plus
`ConstantBias`. On success `VioSystem` calls `VioBackend::initializeSystem()` to install the first
prior and sets the state to `TRACKING_OK`.

### Keyframes and the backend

When a keyframe is accepted, `VioBackend::processKeyframe()` performs:

1. Triangulates new landmarks from feature correspondences between the previous and current
   keyframe, skipping points below `minTriangulationParallaxPx`;
2. Computes reprojection residuals against the preintegrated predicted pose, temporarily rejecting
   observations above `maxReprojectionErrorPreopt`;
3. Inserts `ProjectionFactor`s and runs the smoother update, with `extraSmootherIterations`
   additional iterations;
4. Recomputes residuals against the optimized pose: observations above `maxReprojectionError` are
   marked as outliers after a multi-frame streak, and those above `maxReprojectionErrorHard` are
   rejected on a single frame;
5. Runs `HealthMonitor` and records any alert.

Keyframe data also maintains sliding-window size statistics and the fixed-lag duration
`slidingWindowLag`.

### Accuracy evaluation

`VioSystem::evaluate()` computes translation error per keyframe against simulated ground truth and
reports RMSE and maximum error. RMSE below `rmseThreshold` is reported as passing.

---

## Repository layout

```
myslam/
├── CMakeLists.txt            # top-level build, single version source, format targets
├── config/myslam.lua         # default runtime configuration (copied into build/config/)
├── src/
│   ├── CMakeLists.txt        # INTERFACE targets myslam / myslam_lua / myslam_sanitized
│   ├── myslam.hpp.in         # version template expanded by configure_file
│   ├── lua_config.hpp        # Lua configuration loading
│   ├── log_config.hpp        # logging configuration
│   ├── vio/                  # VIO core, no OpenCV dependency
│   └── dataset/              # dataset replay, OpenCV confined to this layer
├── examples/
│   ├── 1_visual_inertial_odometry.cpp   # synthetic run example
│   └── 2_subt_dataset.cpp               # recorded dataset replay
├── tests/
│   ├── test_vio.cpp          # VIO smoke test with accuracy assertions
│   └── test_lua_config.cpp   # parsing, clamping, cross-validation, failure paths
├── docs/
│   ├── architecture.svg      # architecture diagram (English)
│   └── architecture_zh.svg   # architecture diagram (Chinese)
├── scripts/
│   ├── run_subt.sh           # dataset replay script
│   ├── plot_tum.py           # TUM trajectory plotting
│   ├── gen_architecture_svg.py  # generates the diagrams under docs/
│   └── setup-hooks.sh        # install the pre-commit formatting hook
└── thirdparty/              # gtsam / mylog (git submodules)
```

---

## Requirements

| Item | Requirement |
| :--- | :--- |
| CMake | ≥ 3.20 |
| Compiler | Accepts `-std=c++26` (GCC 14+) |
| Build tool | Ninja or Make |
| System libraries | TBB, Boost, Eigen3, Threads |
| LuaJIT | 2.1+, discoverable via `pkg-config`; when missing, Lua support is turned off automatically |
| OpenCV | 4.x (only when `MYSLAM_BUILD_DATASET_TOOLS=ON`) |
| clang-format | 23.1.3 (commit hook and CI both check against this version) |

Install on Ubuntu:

```bash
sudo apt-get install -y build-essential cmake ninja-build pkgconf \
    libtbb-dev libboost-all-dev libeigen3-dev libluajit-5.1-dev
# dataset tools only
sudo apt-get install -y libopencv-dev
# formatter
pipx install "clang-format==23.1.3"
```

---

## Building

```bash
git clone --recursive <repo-url> myslam
cd myslam

cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

The first build compiles GTSAM.

### Build options

| Option | Default | Description |
| :--- | :--- | :--- |
| `MYSLAM_BUILD_EXAMPLES` | `ON` when top-level | Build example programs |
| `MYSLAM_BUILD_TESTS` | `ON` when top-level | Build tests |
| `MYSLAM_BUILD_DATASET_TOOLS` | `OFF` | Build the OpenCV dataset replay tools |
| `MYSLAM_SANITIZE` | `ON` when top-level | Enable ASan/UBSan in Debug |
| `MYSLAM_ENABLE_LUA` | `ON` | Lua configuration support; falls back to `OFF` with a warning when LuaJIT is missing |

When included via `add_subdirectory`, these options default to `OFF`.

### Separate build for dataset tools

Under Debug + ASan, keyframe optimization is slow, so dataset replay uses a separate Release tree:

```bash
cmake -B build-release -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DMYSLAM_SANITIZE=OFF \
    -DMYSLAM_BUILD_TESTS=OFF \
    -DMYSLAM_BUILD_DATASET_TOOLS=ON
cmake --build build-release --target 2_subt_dataset -j"$(nproc)"
```

`scripts/run_subt.sh` defaults to `build-release`.

### Version

The version string appears once, in `project(VERSION ...)`; `configure_file` writes it to
`<build>/src/generated/myslam.hpp`. Rebuild after changing it.

### Formatting

```bash
cmake --build build --target format        # reformat in place
cmake --build build --target format-check  # check only
```

`scripts/setup-hooks.sh` installs a pre-commit hook that rejects unformatted code.

---

## Running

### Simulation

```bash
cd build
./bin/1_visual_inertial_odometry
```

Runs five seconds of simulation with default configuration (IMU 200 Hz / camera 20 Hz) and prints:

```
评估关键帧样本数 : 24
整体平移 RMSE 误差: 0.0365 米
最大轨迹偏差       : 0.0778 米
```

Exit code `0` means RMSE is below `system.rmseThreshold` (default 0.20 m).

Logs are written to a `.log` file in the **current working directory** (managed by mylog's
`FileManager`); the terminal only prints `[info] 已加载配置: ...`. Open the log file for results.

### Explicit configuration

```bash
./bin/1_visual_inertial_odometry                  # reads config/myslam.lua, else built-in defaults
./bin/1_visual_inertial_odometry my_config.lua    # explicit; exit code 1 on load or validation failure
```

---

## Configuration

Runtime parameters come from a Lua script, by default `config/myslam.lua`, copied into
`build/config/` at configure time.

### Sections

| Section | Consumer | Contents |
| :--- | :--- | :--- |
| `vio` | `VioConfig` | camera intrinsics, extrinsics, IMU noise, Huber kernel threshold, keyframe policy, sliding window |
| `system` | `SystemConfig` | camera step, run duration, RMSE threshold |
| `dataset` | `MeiIntrinsics` / `DatasetRunConfig` | dataset intrinsics, replay range, visualization switches |
| `logger` / `fileManager` | mylog | log level, buffers, rotation and retention |

Every field may be omitted; omission keeps the built-in default.

### Loading mechanism

- The script runs in a bare `lua_State` with no standard libraries registered, so
  `io`/`os`/`package`/`debug` are unavailable. An instruction budget (`kInstructionBudget`,
  50 million) is set as a hook.
- Values are parsed into a temporary copy and written back to `ProjectConfig` only after all
  validation passes; any failure leaves the caller's configuration unchanged.
- Out-of-range values are handled two ways: tuning fields are clamped into a safe interval with a
  warning; hard prerequisites such as focal length, pinhole scale, and image dimensions are
  rejected instead of corrected.
- Unregistered keys inside a section are reported individually.
- Cross-field constraints are checked: keyframe interval bounds, relative ordering of reprojection
  thresholds, and the camera/IMU rate relationship.

### Derived quantity

The IMU sampling step is not a configuration field; it derives from `vio.imuRateHz`:

```cpp
const double imuDt = vio::SystemConfig::imuDtFrom(config_.imuRateHz);  // 1 / imuRateHz
```

A leftover `system.imuDt` in an older config produces a migration message.

### Loading from code

```cpp
#include "lua_config.hpp"

myslam::config::ProjectConfig project;
const auto result = myslam::config::load("config/myslam.lua", project, /*applyLogging=*/true);

if (!result) { /* keep built-in defaults; result.error().message explains why */ }
else if (!result->logging) { /* fall back to the mylog C++ API */ }
```

Without linking `myslam::myslam_lua`, the header is a stub whose `load()` returns failure.

---

## Dataset replay

`2_subt_dataset` uses the same VIO components as the simulator; only the sensor source changes.
Expected layout:

```
Handheld1_Folder/
├── cam_0/
│   ├── 0.png, 1.png, ...
│   └── timestamps.txt      # one nanosecond timestamp per line
└── imu/
    └── imu_data.csv        # timestamp, q_x, q_y, q_z, q_w,
                            # ang_vel_{x,y,z}, lin_acc_{x,y,z}
```

### Handling specific to this dataset

**MEI camera model**: the backend reprojects through `gtsam::Cal3_S2`, while the sequence uses the
MEI model. `MeiFeatureTracker` first inverts the radtan distortion, then back-projects to a ray
under the MEI model, and finally projects through virtual pinhole intrinsics scaled by
`pinholeScale`. The virtual intrinsics are mirrored into `VioConfig`.

**IMU timestamp gaps**: consecutive samples separated by more than
`maxImuGapFactor / imuRateHz` are treated as a gap (default 10 sample periods, i.e. 50 ms at
200 Hz; adjustable via `dataset.replay.maxImuGapFactor`). Two rules apply:

- Zero-order hold never spans a gap: there is no IMU data inside it, so integration stops one
  sample period after the last valid sample instead of extrapolating across the missing time;
- The pipeline is rebuilt and re-initialized. Segment count is reported in
  `DatasetRunReport::segments`, resets in `resets`.

Each segment re-initializes from a local origin, so their coordinate frames differ and
**trajectories and point clouds must not be concatenated across segments**. The TUM output marks
segment boundaries with `#` comment lines (`plot_tum.py` skips them), and the report warns when
there is more than one segment.

The preintegration boundary is aligned to the camera timestamp: a camera time usually falls
between two IMU samples, so integration is truncated proportionally at `cameraTime`, and the
straddling sample is only partially consumed with its remainder left for the next frame. The
interval covered by `CombinedImuFactor` therefore ends exactly at the keyframe timestamp.

### Usage

```bash
build-release/bin/2_subt_dataset \
    --data  /path/Handheld1_Folder/cam_0 \
    --imu   /path/Handheld1_Folder/imu/imu_data.csv \
    --stamps /path/Handheld1_Folder/cam_0/timestamps.txt \
    [--output traj.tum] [--start-frame N] [--max-frames N] [--stride N] \
    [--max-features N] [--max-duration S] [--ply cloud.ply] \
    [--viz] [--video out.mp4] [--viz-stride N] [--viz-rate R] \
    [--config config/myslam.lua]
```

`--config` supplies baseline defaults; command-line arguments take precedence. When omitted, it
reads `config/myslam.lua` in the current directory, else uses built-in defaults.

`--fx/--fy/--cx/--cy/--xi/--k1/--k2/--p1/--p2/--width/--height/--pinhole-scale` override MEI
intrinsics individually.

### Visualization

- `--viz`: OpenCV window (needs `DISPLAY`, ESC/q to quit)
- `--video FILE`: encode to video (falls back to `.avi`/MJPG when mp4 is unavailable)
- `--viz-rate R`: `0` full speed, `1` real time, `2` double speed

Layout: the image with features on top (green = tracked from the previous frame, orange = newly
detected); the bottom status bar holds a trajectory minimap on the left and status text on the
right.

### Inspecting trajectories

```bash
scripts/plot_tum.py traj.tum --save traj.png          # 3D / XY top-down / position-velocity panels
scripts/plot_tum.py traj.tum --ply cloud.ply --show
```

PLY clouds open in MeshLab / CloudCompare / Open3D.

---

## Scripts

### `scripts/run_subt.sh`

```bash
scripts/run_subt.sh                          # default sequence, starting at frame 2300
scripts/run_subt.sh --max-frames 0           # replay everything remaining
scripts/run_subt.sh --root /path/Handheld2_Folder
scripts/run_subt.sh --viz                    # live window
scripts/run_subt.sh --video out.avi --ply cloud.ply
scripts/run_subt.sh --plot                   # plot the trajectory afterwards
scripts/run_subt.sh --no-build               # skip the build
```

The default sequence path is `../MASt3R-Fusion/dataset/Handheld1_Folder`, overridable with
`--root` or `SUBT_ROOT`. It starts at frame 2300 (that sequence has exposure jumps and an IMU gap
early on). When `--max-frames` is not given, the script uses `500000`.

### `scripts/plot_tum.py`

Requires `numpy` and `matplotlib`. Produces three panels: 3D trajectory, XY top-down, and
position/velocity over time.

### `scripts/setup-hooks.sh`

Installs `.githooks/pre-commit` (`git config core.hooksPath .githooks`), which checks staged files
with `clang-format --dry-run -Werror`.

---

## Tests

```bash
ctest --test-dir build --output-on-failure
```

| Test | Coverage |
| :--- | :--- |
| `myslam_test` | Public API usable, keyframes produced, RMSE within threshold, derived step valid |
| `myslam_config_test` | Parsing, clamping, cross-validation, unknown keys, quaternion normalization, failure paths, logging section effect, default-config self-consistency |

CI (`.github/workflows/ci.yml`) runs format checking, a build, and the suite on Ubuntu with
`g++-14`, pinning `clang-format` 23.1.3.

---

## Engineering conventions

Conventions come from the mylog submodule; the full statement is in the project's
`instructions.md`.

- C++26, 4-space indent, Allman braces, 120 columns
- Types `CamelCase`; functions and locals `camelBack`; members suffixed `_`
- Constants `k` prefix + `CamelCase`; enums and macros `UPPER_CASE`; namespaces `lower_case`
- Build switches prefixed `MYSLAM_*`; test target `myslam_test`
- Doxygen comments; clang-format required before committing

---

## Current limitations

- No loop closure or global relocalization; error accumulates over time.
- No map persistence; landmarks are reclaimed as the marginalization window advances.
- Monocular camera plus IMU only; no multi-camera, depth, or GNSS fusion.
- No `install()` or `export()` rules. `myslam` is an INTERFACE target, `INSTALL_INTERFACE` is a
  placeholder, and the generated header is not installed.
- Configuration loads once at startup; no runtime hot reload.
- Logs are written to the current working directory rather than a project-managed location.
- If a prefix that shadows the system `libfontconfig` is active (e.g. conda), the dataset tools
  fail at runtime with `undefined symbol: FcConfigSetDefaultSubstitute`; work around it with
  `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu`.
