% params.m
% Mission parameters for the variable-tilt quadcopter simulation.
% Run this before build_model.m and run_simulation.m.
%
% The first time you run it, it asks for three mission inputs interactively.
% On subsequent runs (e.g. when rebuilding the model), it skips the prompts
% if those variables are already in the workspace. To re-enter them, clear
% the three variables first: clear target_xy h_cruise v_cruise

if ~exist('target_xy','var') || ~exist('h_cruise','var') || ~exist('v_cruise','var')
    fprintf('\n--- Mission inputs (press Enter to accept the default) ---\n');

    % Target waypoint
    while true
        s = input('  Target waypoint [x y] in metres [default [25 10], max +/-100 each]: ', 's');
        if isempty(s), target_xy = [25 10]; break; end
        try v = str2num(s); catch, v = []; end %#ok<ST2NM>
        if numel(v)==2 && all(isfinite(v)) && all(abs(v)<=100)
            target_xy = v(:).'; break;
        end
        fprintf('    Invalid input. Enter two numbers, e.g. "30 15" or "[30 15]".\n');
    end

    % Cruise altitude
    while true
        s = input('  Cruise altitude [m] [default 5, range 1..30]: ', 's');
        if isempty(s), h_cruise = 5; break; end
        v = str2double(s);
        if ~isnan(v) && isreal(v) && v>=1 && v<=30, h_cruise = v; break; end
        fprintf('    Invalid. Enter a number between 1 and 30.\n');
    end

    % Cruise forward speed
    while true
        s = input('  Cruise forward speed [m/s] [default 8, range 0.5..12]: ', 's');
        if isempty(s), v_cruise = 8; break; end
        v = str2double(s);
        if ~isnan(v) && isreal(v) && v>=0.5 && v<=12, v_cruise = v; break; end
        fprintf('    Invalid. Enter a number between 0.5 and 12.\n');
    end
end

% Sanity check even on reuse, in case something changed externally
assert(isnumeric(target_xy) && numel(target_xy)==2 && all(abs(target_xy)<=100), ...
    'target_xy must be a 2-element vector with each component in +/-100 m.');
assert(h_cruise >= 1 && h_cruise <= 30, 'h_cruise must be in [1, 30] m.');
assert(v_cruise >= 0.5 && v_cruise <= 12, 'v_cruise must be in [0.5, 12] m/s.');

% Guidance tolerances — not user-configurable, but easy to tweak here
psi_tol            = deg2rad(8);   % heading error before we allow forward thrust [rad]
arrive_tol         = 0.40;         % horizontal radius considered "arrived" [m]
min_approach_speed = 0.15;         % slowest we let the drone creep on final approach [m/s]

% Phased mission timing (takeoff -> align -> cruise -> hover -> descend -> land)
t_takeoff    = 8.0;   % ramp duration for the altitude climb [s]
hover_time   = 1.5;   % how long we hold position over the target [s]
descend_rate = 1.2;   % altitude slew rate during descent [m/s]
h_land       = 0.0;   % target altitude at touchdown [m]
decel_frac   = 0.20;  % fraction of cruise distance used for braking
psi_lock_dist= 1.0;   % freeze heading once we get this close to target [m]
kp_pos_hold  = 0.5;   % P-gain for station-keeping once in hover [1/s]
psi_hold_t   = 0.5;   % heading must hold steady this long before cruise starts [s]
h_tol        = 0.15;  % altitude band to consider "at cruise height" [m]

% Cap on reverse tilt speed when recovering from overshoot, scaled to cruise speed
v_recover_max = 0.25 * v_cruise;

% Keep h_des as an alias so Simulink blocks can reference it by name
h_des = h_cruise;

% =============================================================
%  Physical model parameters
% =============================================================
m   = 1.5;    % total mass [kg]
g   = 9.81;   % gravity [m/s^2]
L   = 0.225;  % arm length (centre to rotor) [m]
Ixx = 0.02;  Iyy = 0.02;  Izz = 0.04;   % moments of inertia [kg.m^2]
k_q = 0.01;   % reactive yaw torque per unit thrust [N.m / N]

% Quadratic aerodynamic drag in the body frame
Cd_xy = 0.05;   % horizontal drag coefficient [N.s^2/m^2]
Cd_z  = 0.03;   % vertical drag coefficient [N.s^2/m^2]

% Actuator limits
T_max         = 15;             % max thrust per rotor [N]
T_rate_max    = 30;             % max thrust slew rate [N/s]
beta_max      = deg2rad(60);    % max tilt angle [rad]
beta_rate_max = deg2rad(45);    % max tilt rate [rad/s]
alt_margin    = 0.05;           % vertical-thrust safety margin (5 %)

% =============================================================
%  Sim time — scales automatically with mission distance
% =============================================================
dist_to_goal = hypot(target_xy(1), target_xy(2));
yaw_time     = pi / 1.0;   % budget for the alignment rotation

% The actual cruise is slower than commanded v_cruise because of the
% altitude-first thrust cap, ease-in, braking, and terminal crawl,
% so we add a fixed overhead on top of the naive leg time.
travel_time  = dist_to_goal / max(v_cruise, 0.5) + 15;
descend_time = max(h_cruise - h_land, 0) / max(descend_rate, 0.05);
mission_time = t_takeoff + yaw_time + travel_time + hover_time + descend_time + 12;
sim_time     = ceil(mission_time);
sim_time     = min(max(sim_time, 15), 180);   % clamp to [15, 180] s
assert(mission_time <= 180, ...
    'Mission exceeds 180 s budget. Increase v_cruise or move target_xy closer.');

% =============================================================
%  Forward-velocity PID  (vx_des -> Fx_des)
% =============================================================
Kp_vx = 1.5;  Ki_vx = 0.4;  Kd_vx = 0.3;  Nf_vx = 20;

% Work out how much horizontal force each rotor can produce without stealing
% vertical authority. Altitude always has priority over horizontal speed.
Tz_hold      = m*g*(1+alt_margin)/4;
Tx_per_rotor = sqrt(max(T_max^2 - Tz_hold^2, 0));
Fx_pid_max   =  4*Tx_per_rotor;
Fx_pid_min   = -Fx_pid_max;

% =============================================================
%  Altitude PID  (outer loop -> Fz_des)
% =============================================================
Kp_alt = 4.0;  Ki_alt = 0.8;  Kd_alt = 8.0;  Nf_alt = 30;
Fz_pid_max =  4*T_max - m*g;
Fz_pid_min = -m*g;

% =============================================================
%  Attitude PIDs  (Euler angles -> body moments)
% =============================================================
% Roll and pitch use pure PD — no integrator, because we want the drone to
% tilt naturally with the allocator and not fight its own trim.
Kp_phi = 0.8;  Ki_phi = 0.0;  Kd_phi = 0.4;  Nf_att = 30;
Kp_th  = 0.8;  Ki_th  = 0.0;  Kd_th  = 0.4;

% Yaw uses a small integrator to hold heading precisely during cruise.
% The saturation limits keep the allocator from spending too much vertical
% thrust differential on yaw correction.
Kp_psi = 1.2;  Ki_psi = 0.10;  Kd_psi = 0.5;
Mz_pid_max =  6 * k_q * (m*g/4);
Mz_pid_min = -Mz_pid_max;
