# Variable-tilt quadcopter simulation

A 6-DOF quadcopter flight simulator in MATLAB/Simulink where each rotor can tilt on its own about its lateral axis. Tilting the rotors gives the drone horizontal thrust without pitching the body, so altitude and forward speed can be controlled separately.

The simulation flies a full point-to-point mission: vertical takeoff, turning to face the target, forward cruise with bang-bang braking, hover, and a controlled descent to landing.

## Features

- Each of the 4 rotors tilts ±60° about its lateral axis, so one actuator gives both vertical and horizontal thrust.
- The drone holds cruise altitude without pitching. The altitude PID and the velocity PID run independently.
- A 6-state guidance state machine sequences the mission: takeoff, align, cruise, hover, descend, land.
- The thrust allocator meets the vertical demand first. Horizontal thrust gets whatever rotor capacity is left.
- Cascaded PID loops control altitude, velocity, roll, pitch and yaw, with configurable gains and anti-windup.
- In cruise, on/off (bang-bang) deceleration logic stops the drone at the target.
- The Simulink model is built by a script. The `.slx` is committed and opens directly, and `build_model.m` can regenerate it.
- An animated 4-pane dashboard shows the 3D flight path, the drone geometry, tilt gauges and a top-down map, live or from recorded data.
- A performance report prints more than 40 metrics, including overshoot, RMS errors, actuator saturation and phase timing.

## Requirements

| Toolbox | Used for |
|---|---|
| Simulink | Model execution |
| Aerospace Blockset (`aerolib6dof2`) | 6DOF Euler-angles rigid-body block |
| Stateflow | MATLAB Function blocks (guidance FSM, allocator) |
| Control System Toolbox (`slpidlib`) | PID Controller blocks |

MATLAB R2023b or newer is recommended.

## Folder structure

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

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/MiasJHendrikse/tilt-quad-sim.git
   cd tilt-quad-sim
   ```

2. Open MATLAB (R2023b or newer) and set the working directory to the cloned folder:

   ```matlab
   cd('path/to/tilt-quad-sim')
   ```

3. Check that the four required toolboxes are installed. Each of these should be listed:

   ```matlab
   ver('simulink'), ver('aero'), ver('control'), ver('stateflow')
   ```

4. Optionally, regenerate the model. `tilt_quad_sim.slx` is already in the repo, so you only need this if you change `build_model.m`:

   ```matlab
   build_model
   ```

## Quick start

```matlab
% 1. Set mission parameters (prompts you for target, altitude, speed on first run)
params

% 2. Run the simulation and open the post-processing dashboard
run_simulation
```

After the first run you can call `run_simulation` on its own. It calls `params`, which sees that the variables are already in the workspace and skips the prompts.

To enter new mission parameters, clear the old ones first:

```matlab
clear target_xy h_cruise v_cruise
run_simulation
```

## Running the simulation

### From the MATLAB script

`run_simulation.m` is the single entry point. It:

1. calls `params`, or skips it if the parameters are already set
2. builds the model if `tilt_quad_sim.slx` does not exist
3. runs the Simulink simulation
4. asks whether to open the animation, the metrics charts and the stats report

### From Simulink

You can also open `tilt_quad_sim.slx` in Simulink and press Run. The model needs workspace variables, so run `params` in MATLAB first:

```matlab
params          % loads gains, physical constants, and mission parameters
% then open tilt_quad_sim.slx and press Run in Simulink
```

In this case the post-processing doesn't run by itself. Call `plot_animation`, `plot_metrics` or `mission_stats` afterwards.

### Post-processing tools

| Script | What it shows |
|---|---|
| `plot_animation` | Animated 4-pane dashboard; runs after the simulation or replays logged data |
| `plot_metrics` | Static 3×3 tiled charts: altitude, speed, ground track, attitude, body rates, thrust, tilt, mission state, controller demands |
| `mission_stats` | Prints ~40 performance metrics to the Command Window and saves a `stats` struct to the workspace |

## Examples

### Short hop at low altitude

```matlab
% At the prompts in params:
%   Target [x, y] (m): [5, 0]
%   Cruise altitude (1–30 m): 3
%   Cruise speed (0.5–12 m/s): 1.5

run_simulation
```

Expected output from `mission_stats`:

```
=== Mission Performance Report ===
Altitude overshoot:       0.18 m  (6.1%)
Cruise speed error (RMS): 0.04 m/s
Position accuracy:        0.11 m
Total flight time:        42.3 s
Phases completed:         6 / 6  [PASS]
```

### Longer cruise

```matlab
clear target_xy h_cruise v_cruise
% At the prompts:
%   Target [x, y] (m): [20, 10]
%   Cruise altitude (1–30 m): 10
%   Cruise speed (0.5–12 m/s): 5

run_simulation
```

The guidance state machine lengthens the simulation to suit the distance and speed. After the run, `plot_animation` replays the flight path.

### Exporting the animation to video

```matlab
% In plot_animation.m, set the export flag before running:
export_video = true;   % saves tilt_quad_animation.mp4

plot_animation
```

## How it works

### Tilt actuation

Each rotor sits on a tilt servo that rotates it forward or backward about the drone's y-axis. At zero tilt the rotor pushes straight up. Tilted by an angle β, it produces:

- a vertical component `T · cos(β)`
- a horizontal component `T · sin(β)`

So the drone can accelerate horizontally without changing its body pitch.

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

The allocator meets the vertical demand first. Each rotor reserves enough vertical thrust to hold the drone up, and the horizontal thrust is capped at what remains.

### Guidance phases

| State | Name | What happens |
|---|---|---|
| 1 | TAKEOFF | Smoothstep altitude ramp to cruise height |
| 2 | ALIGN | Rotate to face the target (yaw only, no translation) |
| 3 | CRUISE | Forward flight with bang-bang braking as it approaches |
| 4 | HOVER | Hold position over the target for a short dwell |
| 5 | DESCEND | Lower slowly to landing height |
| 6 | LANDED | Soft position hold; the simulation stops automatically |

### Physical model

| Parameter | Value |
|---|---|
| Mass | 1.5 kg |
| Arm length | 0.225 m |
| Ixx / Iyy / Izz | 0.02 / 0.02 / 0.04 kg·m² |
| Max thrust per rotor | 15 N |
| Max tilt angle | ±60° |
| Drag coefficients | Cd\_xy = 0.05, Cd\_z = 0.03 N·s²/m² |

## Tuning

All the gains are in `params.m`, grouped into sections:

- altitude PID: `Kp_alt`, `Ki_alt`, `Kd_alt`
- velocity PID: `Kp_vx`, `Ki_vx`, `Kd_vx`
- attitude PIDs: `Kp_phi/th/psi`, `Kd_phi/th/psi`
- mission timing: `t_takeoff`, `hover_time`, `descend_rate`, `decel_frac`
- actuator limits: `T_max`, `beta_max`, `T_rate_max`, `beta_rate_max`

## Notes and limitations

- `tilt_quad_sim.slx` is committed. Run `build_model` only if you want to regenerate it after changing `build_model.m`.
- Simulink creates the `slprj/` build cache automatically, and you can delete it.
- If `build_model` reports a missing library, check that the Aerospace Blockset and Control System Toolbox are installed.
- The 6DOF block uses Euler angles, so gimbal lock is possible near ±90° pitch. Normal flight stays far from that.
- Drag is modelled only as quadratic damping in the body frame. There is no blade flapping, ground effect or motor dynamics.
- MATLAB runs the simulation on a single thread, so very long missions (over 200 s) can be slow.

## License

MIT. You can use, modify and distribute it with attribution.
