# Variable-Tilt Quadcopter Simulation

A 6-DOF quadcopter flight simulator in MATLAB/Simulink where each rotor can tilt independently around its lateral axis. This lets the drone generate horizontal thrust directly — without pitching the whole body — so altitude and forward speed are decoupled and controlled separately.

The simulation covers a full point-to-point mission: vertical takeoff, heading alignment, forward cruise with bang-bang braking, hover, and a controlled descent to landing.

---

## Features

- **Variable-tilt rotors** — each of the 4 rotors tilts ±60° around its lateral axis, producing simultaneous vertical and horizontal thrust from a single actuator
- **Decoupled altitude and speed control** — the drone maintains cruise altitude without pitching; altitude PID and velocity PID run independently
- **6-state guidance FSM** — autonomous mission sequencer: takeoff → align → cruise → hover → descend → land
- **Altitude-first thrust allocator** — vertical demand is always satisfied first; horizontal thrust uses whatever rotor capacity remains
- **Cascade PID control** — altitude, velocity, roll, pitch, and yaw loops with configurable gains and anti-windup
- **Bang-bang braking** — cruise phase uses on/off deceleration logic for efficient stopping at the target
- **Programmatic Simulink model** — the `.slx` is committed and can be opened directly; it can also be regenerated from `build_model.m`
- **Animated 4-pane dashboard** — real-time (or recorded) visualisation of 3D flight path, drone geometry, tilt gauges, and top-down map
- **Quantitative performance report** — 40+ metrics: overshoot, RMS errors, actuator saturation, phase timing

---

## Requirements

| Toolbox | Used for |
|---|---|
| Simulink | Model execution |
| Aerospace Blockset (`aerolib6dof2`) | 6DOF Euler-angles rigid-body block |
| Stateflow | MATLAB Function blocks (guidance FSM, allocator) |
| Control System Toolbox (`slpidlib`) | PID Controller blocks |

MATLAB R2023b or newer is recommended.

---

## Folder Structure

```
tilt-quad-sim/
├── README.md            — this file
├── .gitignore           — excludes slprj/, *.mat, and generated .slx (tilt_quad_sim.slx is committed)
├── params.m             — mission parameters, PID gains, physical constants
├── build_model.m        — generates tilt_quad_sim.slx using the Simulink API
├── tilt_quad_sim.slx    — Simulink model (committed; can also be regenerated via build_model.m)
├── run_simulation.m     — top-level entry point; calls params, build, sim, post-processing
├── plot_animation.m     — animated 4-pane dashboard (3D view, drone geometry, tilt gauges, map)
├── plot_metrics.m       — static 3×3 tiled time-history charts
├── figure_defs.m        — shared chart definitions used by plot_metrics.m
└── mission_stats.m      — prints quantitative performance report to Command Window
```

---

## Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/<your-username>/tilt-quad-sim.git
   cd tilt-quad-sim
   ```

2. **Open MATLAB** (R2023b or newer) and set the working directory to the cloned folder

   ```matlab
   cd('path/to/tilt-quad-sim')
   ```

3. **Verify toolboxes** — run the following and confirm the four required toolboxes are listed:

   ```matlab
   ver('simulink'), ver('aero'), ver('control'), ver('stateflow')
   ```

4. **Optional — regenerate the model** — `tilt_quad_sim.slx` is already in the repo; you only need this if you modify `build_model.m`:

   ```matlab
   build_model
   ```

---

## Quick Start

```matlab
% 1. Set mission parameters (prompts you for target, altitude, speed on first run)
params

% 2. Run the simulation and open the post-processing dashboard
run_simulation
```

On subsequent runs, just call `run_simulation` — `params` is called automatically and detects that variables are already in the workspace.

Alternatively, you can open `tilt_quad_sim.slx` directly in Simulink and press **Run** — but you must run `params` in MATLAB first so the workspace variables are loaded.

To re-enter mission parameters, clear them first:

```matlab
clear target_xy h_cruise v_cruise
run_simulation
```

---

## How to Run / Use

### Running the Simulink model directly

You can open `tilt_quad_sim.slx` in Simulink and press **Run** without going through `run_simulation.m`. **You must run `params` in MATLAB first** to load the workspace variables the model depends on:

```matlab
params          % loads gains, physical constants, and mission parameters
% then open tilt_quad_sim.slx and press Run in Simulink
```

Post-processing (animation, metrics, stats) still requires running `plot_animation`, `plot_metrics`, or `mission_stats` manually afterwards.

### Running via the MATLAB script

`run_simulation.m` is the single entry point. It:

1. Calls `params` (or skips if parameters are already set)
2. Builds the model if `tilt_quad_sim.slx` does not exist
3. Runs the Simulink simulation
4. Prompts you to open the animation, metrics, and/or stats report

### Post-processing tools

| Script | What it shows |
|---|---|
| `plot_animation` | Animated 4-pane dashboard — runs after simulation or can replay from logged data |
| `plot_metrics` | Static 3×3 tiled charts: altitude, speed, ground track, attitude, body rates, thrust, tilt, mission state, controller demands |
| `mission_stats` | Prints ~40 performance metrics to the Command Window and saves a `stats` struct to the workspace |

---

## Example Usage

### Example 1 — short hop, low altitude

```matlab
% At the prompts in params:
%   Target [x, y] (m): [5, 0]
%   Cruise altitude (1–30 m): 3
%   Cruise speed (0.5–12 m/s): 1.5

run_simulation
```

Expected output (mission_stats):

```
=== Mission Performance Report ===
Altitude overshoot:       0.18 m  (6.1%)
Cruise speed error (RMS): 0.04 m/s
Position accuracy:        0.11 m
Total flight time:        42.3 s
Phases completed:         6 / 6  [PASS]
```

### Example 2 — long-range cruise

```matlab
clear target_xy h_cruise v_cruise
% At the prompts:
%   Target [x, y] (m): [20, 10]
%   Cruise altitude (1–30 m): 10
%   Cruise speed (0.5–12 m/s): 5

run_simulation
```

The guidance FSM will increase simulation duration automatically (scales with distance and speed). After the run, use `plot_animation` to replay the flight path.

### Example 3 — export animation to video

```matlab
% In plot_animation.m, set the export flag before running:
export_video = true;   % saves tilt_quad_animation.mp4

plot_animation
```

---

## How It Works

### Tilt actuation

Each rotor sits on a tilt servo that rotates it forward/backward (around the drone's y-axis). At zero tilt the rotor pushes straight up. When tilted by angle β, it produces:

- Vertical component: `T · cos(β)`
- Horizontal component: `T · sin(β)`

This means the drone can accelerate horizontally without changing its body pitch angle.

### Control architecture

```
Guidance FSM  →  velocity / heading / altitude setpoints
                ↓
Altitude PID  →  Fz_des     ← outer loop, highest priority
Velocity PID  →  Fx_des     ← horizontal, capped by altitude headroom
Attitude PIDs →  Mx,My,Mz   ← roll/pitch levelling + yaw tracking
                ↓
Allocator     →  T(4), β(4) ← maps force/moment demands to per-rotor thrust and tilt
                ↓
6DOF Plant    →  position, velocity, Euler angles, body rates
```

The allocator satisfies the vertical demand first. Horizontal thrust is capped to whatever capacity remains after each rotor has reserved enough vertical component to hold the drone up.

### Guidance phases

| State | Name | What happens |
|---|---|---|
| 1 | TAKEOFF | Smoothstep altitude ramp to cruise height |
| 2 | ALIGN | Rotate to face the target (yaw only, no translation) |
| 3 | CRUISE | Forward flight with bang-bang braking as it approaches |
| 4 | HOVER | Station-keep over target for a brief dwell |
| 5 | DESCEND | Slowly lower altitude to landing height |
| 6 | LANDED | Soft position hold; simulation stops automatically |

### Physical model

| Parameter | Value |
|---|---|
| Mass | 1.5 kg |
| Arm length | 0.225 m |
| Ixx / Iyy / Izz | 0.02 / 0.02 / 0.04 kg·m² |
| Max thrust per rotor | 15 N |
| Max tilt angle | ±60° |
| Drag coefficients | Cd\_xy = 0.05, Cd\_z = 0.03 N·s²/m² |

---

## Tuning

All gains are in `params.m` under clearly labelled sections:

- **Altitude PID** — `Kp_alt`, `Ki_alt`, `Kd_alt`
- **Velocity PID** — `Kp_vx`, `Ki_vx`, `Kd_vx`
- **Attitude PIDs** — `Kp_phi/th/psi`, `Kd_phi/th/psi`
- **Mission timing** — `t_takeoff`, `hover_time`, `descend_rate`, `decel_frac`
- **Actuator limits** — `T_max`, `beta_max`, `T_rate_max`, `beta_rate_max`

---

## Notes / Limitations

- `tilt_quad_sim.slx` is committed. Run `build_model` only if you want to regenerate it after modifying `build_model.m`.
- The `slprj/` folder (Simulink build cache) is generated automatically and safe to delete.
- If MATLAB complains about a missing library on `build_model`, make sure the Aerospace Blockset and Control System Toolbox are installed.
- The 6DOF model uses Euler angles — gimbal lock can occur near ±90° pitch, though normal flight stays well within safe limits.
- Drag is modelled as body-frame quadratic damping only; no blade flapping, ground effect, or motor dynamics.
- The simulation runs in single-threaded MATLAB; very long missions (>200 s) may be slow to simulate.

---

## License

MIT — free to use, modify, and distribute with attribution.
