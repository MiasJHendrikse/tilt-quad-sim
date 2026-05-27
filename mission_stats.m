% mission_stats.m
% Quantitative performance report for the variable-tilt quadcopter.
% Run after a simulation so that `out` is in the workspace.
%
% Prints a sectioned report to the Command Window and saves a `stats`
% struct to the base workspace for any further analysis.

assert(exist('out','var') == 1, 'mission_stats: run a simulation first (no `out` in workspace).');

% Pull params from workspace, fall back to sensible defaults if missing
h_cruise = ds_getvar('h_cruise',  5);
v_cruise = ds_getvar('v_cruise',  8);
h_land   = ds_getvar('h_land',    0);
T_max    = ds_getvar('T_max',    15);
beta_max = ds_getvar('beta_max', deg2rad(60));
target   = ds_getvar('target_xy', [25 10]);

% Pull logged signals
[t ,Xe] = ds_sig(out,'Xe');    alt=Xe(:,3); posN=Xe(:,1); posE=Xe(:,2);
[~ ,Ve] = ds_sig(out,'Ve');    gspd=hypot(Ve(:,1),Ve(:,2)); vz=Ve(:,3);
[~ ,Vb] = ds_sig(out,'Vb');    vbx=Vb(:,1);
[~ ,Eu] = ds_sig(out,'Euler'); phi=Eu(:,1); th=Eu(:,2); psi=Eu(:,3);
[~ ,W ] = ds_sig(out,'PQR');
[tT,T ] = ds_sig(out,'Tlog');
[tB,B ] = ds_sig(out,'BetaLog');
[tH,Hd] = ds_sig(out,'HDesLog');
[tV,Vd] = ds_sig(out,'VxDesLog');
[tP,Pd] = ds_sig(out,'PsiDesLog');
[tS,St] = ds_sig(out,'StateLog'); St=round(St);
[tF,Fx] = ds_sig(out,'FxDes');
[~ ,Fz] = ds_sig(out,'FzDes');

% Interpolate setpoints onto the main time vector
Hd_i = interp1(tH, Hd, t, 'linear', 'extrap');
Vd_i = interp1(tV, Vd, t, 'linear', 'extrap');
Pd_i = interp1(tP, Pd, t, 'linear', 'extrap');
St_i = interp1(tS, St, t, 'previous', 'extrap');

inCruise = (St_i == 3);
postAlign= (St_i >= 2);
preDesc  = (St_i <= 4);
descIdx  = find(St_i >= 5, 1);
tEnd     = t(end);

s = struct();

% ===================== ALTITUDE =====================
peakAlt              = max(alt(preDesc | isempty(descIdx)));
s.alt_overshoot_m    = max(0, peakAlt - h_cruise);
s.alt_overshoot_pct  = 100 * s.alt_overshoot_m / max(h_cruise, eps);
s.alt_rms_err_all    = sqrt(mean((alt - Hd_i).^2));
s.alt_rms_err_cruise = ds_rms(alt(inCruise) - Hd_i(inCruise));
i90 = find(alt >= 0.90*h_cruise, 1);  s.alt_rise_t90 = ds_pick(t, i90);
band    = 0.02 * h_cruise;
outBand = preDesc & (abs(alt - h_cruise) > band);
li      = find(outBand, 1, 'last');
s.alt_settle_time  = ds_pick(t, ds_nextidx(li, numel(t)));
s.max_climb_rate   =  max(vz);
s.max_descent_rate = -min(vz);

if ~isempty(descIdx)
    tdIdx = descIdx - 1 + find(alt(descIdx:end) <= h_land + 0.05, 1);
else
    tdIdx = [];
end
s.touchdown_time = ds_pick(t, tdIdx);
s.final_alt_m    = alt(end);
s.landing_err_m  = abs(alt(end) - h_land);

% ===================== SPEED / VELOCITY =====================
s.peak_fwd_speed    = max(vbx);
s.peak_ground_speed = max(gspd);
s.pct_of_vcruise    = 100 * s.peak_fwd_speed / max(v_cruise, eps);
if any(inCruise)
    s.cruise_mean_speed  = mean(vbx(inCruise));
    s.cruise_rms_spd_err = ds_rms(vbx(inCruise) - Vd_i(inCruise));
    ci = find(inCruise, 1);
    j  = find(inCruise & vbx >= 0.9*v_cruise, 1);
    s.t_reach_90pct_vc = ds_pick(t, j) - ds_pick(t, ci);
else
    [s.cruise_mean_speed, s.cruise_rms_spd_err, s.t_reach_90pct_vc] = deal(NaN);
end

% ===================== POSITION / GUIDANCE =====================
dN = posN - target(1); dE = posE - target(2); rng = hypot(dN, dE);
s.final_pos_err_m    = rng(end);
s.closest_approach_m = min(rng);
hd0  = atan2(target(2)-posE(1), target(1)-posN(1));
proj = (posN-target(1))*cos(hd0) + (posE-target(2))*sin(hd0);
s.max_overshoot_past_target_m = max(0, max(proj));
pathLen = sum(hypot(diff(posN), diff(posE)));
straight= hypot(target(1)-posN(1), target(2)-posE(1));
s.path_length_m   = pathLen;
s.path_efficiency = straight / max(pathLen, eps);

% ===================== HEADING / ATTITUDE =====================
yawErr = atan2(sin(psi - Pd_i), cos(psi - Pd_i));
s.yaw_rms_err_deg     = rad2deg(ds_rms(yawErr));
s.yaw_rms_err_aligned = rad2deg(ds_rms(yawErr(postAlign)));
s.yaw_max_err_deg     = rad2deg(max(abs(yawErr)));
s.max_roll_deg   = rad2deg(max(abs(phi)));
s.max_pitch_deg  = rad2deg(max(abs(th)));
s.rms_roll_deg   = rad2deg(ds_rms(phi));
s.rms_pitch_deg  = rad2deg(ds_rms(th));
s.max_p_dps = rad2deg(max(abs(W(:,1))));
s.max_q_dps = rad2deg(max(abs(W(:,2))));
s.max_r_dps = rad2deg(max(abs(W(:,3))));

% ===================== ACTUATORS / CONTROL =====================
s.max_thrust_N    = max(T(:));
s.pct_of_Tmax     = 100 * s.max_thrust_N / max(T_max, eps);
anyHot            = any(T >= 0.99*T_max, 2);
s.thrust_sat_frac = mean(anyHot);
s.max_tilt_deg    = rad2deg(max(abs(B(:))));
s.pct_of_betamax  = 100 * max(abs(B(:))) / max(beta_max, eps);
s.ctrl_effort_Fx  = trapz(tF, abs(Fx));
s.ctrl_effort_Fz  = trapz(tF, abs(Fz));
s.thrust_impulse  = trapz(tT, sum(T, 2));

% ===================== MISSION TIMING =====================
names  = {'TAKEOFF','ALIGN','CRUISE','HOVER','DESCEND','LANDED'};
entryT = nan(1,6); durT = nan(1,6);
for k = 1:6
    idx = find(St == k, 1);
    if ~isempty(idx), entryT(k) = tS(idx); end
end
for k = 1:6
    if ~isnan(entryT(k))
        nxt  = entryT(k+1:end); nxt = nxt(~isnan(nxt));
        endk = min([nxt, tEnd]);
        durT(k) = endk - entryT(k);
    end
end
s.phase_entry_s  = entryT;
s.phase_dur_s    = durT;
s.mission_time_s = tEnd;

assignin('base', 'stats', s);

% ===================== PRINT REPORT =====================
L  = @(lbl,val,unit,f) fprintf(['  %-34s ' f ' %s\n'], lbl, val, unit);
hr = @() fprintf('%s\n', repmat('-',1,60));
fprintf('\n========== DRONE PERFORMANCE STATS ==========\n');
fprintf('Mission -> target [%g %g] m | h_cruise=%g m | v_cruise=%g m/s\n', ...
    target(1), target(2), h_cruise, v_cruise);
hr(); fprintf('ALTITUDE\n');
L('Takeoff/cruise overshoot',    s.alt_overshoot_m,    'm',   '%8.3f');
L('  as %% of h_cruise',         s.alt_overshoot_pct,  '%',   '%8.2f');
L('Rise time to 90%% h_cruise',  s.alt_rise_t90,       's',   '%8.2f');
L('Settling time (2%% band)',     s.alt_settle_time,    's',   '%8.2f');
L('RMS alt error (whole)',        s.alt_rms_err_all,    'm',   '%8.3f');
L('RMS alt error (cruise)',       s.alt_rms_err_cruise, 'm',   '%8.3f');
L('Max climb rate',               s.max_climb_rate,     'm/s', '%8.3f');
L('Max descent rate',             s.max_descent_rate,   'm/s', '%8.3f');
L('Touchdown time',               s.touchdown_time,     's',   '%8.2f');
L('Final altitude',               s.final_alt_m,        'm',   '%8.4f');
L('Landing error vs h_land',      s.landing_err_m,      'm',   '%8.4f');
hr(); fprintf('SPEED\n');
L('Peak forward speed (V_b,x)',   s.peak_fwd_speed,       'm/s', '%8.3f');
L('Peak ground speed',            s.peak_ground_speed,    'm/s', '%8.3f');
L('  as %% of v_cruise',          s.pct_of_vcruise,       '%',   '%8.2f');
L('Cruise mean speed',            s.cruise_mean_speed,    'm/s', '%8.3f');
L('Cruise RMS speed error',       s.cruise_rms_spd_err,   'm/s', '%8.3f');
L('Time to reach 90%% v_cruise',  s.t_reach_90pct_vc,     's',   '%8.2f');
hr(); fprintf('POSITION / GUIDANCE\n');
L('Final position error',         s.final_pos_err_m,              'm',  '%8.3f');
L('Closest approach to target',   s.closest_approach_m,           'm',  '%8.3f');
L('Max overshoot past target',    s.max_overshoot_past_target_m,  'm',  '%8.3f');
L('Path length',                  s.path_length_m,                'm',  '%8.2f');
L('Path efficiency (1=ideal)',    s.path_efficiency,              '',   '%8.3f');
hr(); fprintf('HEADING / ATTITUDE\n');
L('Yaw RMS error (whole)',        s.yaw_rms_err_deg,     'deg', '%8.3f');
L('Yaw RMS error (post-align)',   s.yaw_rms_err_aligned, 'deg', '%8.3f');
L('Yaw max error',                s.yaw_max_err_deg,     'deg', '%8.3f');
L('Max |roll|',                   s.max_roll_deg,        'deg', '%8.3f');
L('Max |pitch|',                  s.max_pitch_deg,       'deg', '%8.3f');
L('RMS roll',                     s.rms_roll_deg,        'deg', '%8.3f');
L('RMS pitch',                    s.rms_pitch_deg,       'deg', '%8.3f');
L('Max |p| (roll rate)',          s.max_p_dps,           'deg/s','%8.2f');
L('Max |q| (pitch rate)',         s.max_q_dps,           'deg/s','%8.2f');
L('Max |r| (yaw rate)',           s.max_r_dps,           'deg/s','%8.2f');
hr(); fprintf('ACTUATORS / CONTROL\n');
L('Max rotor thrust',             s.max_thrust_N,        'N',   '%8.3f');
L('  as %% of T_max',             s.pct_of_Tmax,         '%',   '%8.2f');
L('Thrust-saturation fraction',   100*s.thrust_sat_frac, '%',   '%8.2f');
L('Max tilt |beta|',              s.max_tilt_deg,        'deg', '%8.2f');
L('  as %% of beta_max',          s.pct_of_betamax,      '%',   '%8.2f');
L('Control effort int|Fx|dt',     s.ctrl_effort_Fx,      'N.s', '%8.1f');
L('Control effort int|Fz|dt',     s.ctrl_effort_Fz,      'N.s', '%8.1f');
L('Thrust impulse int(sumT)dt',   s.thrust_impulse,      'N.s', '%8.1f');
hr(); fprintf('MISSION TIMING\n');
for k = 1:6
    if ~isnan(s.phase_dur_s(k))
        fprintf('  %-10s enter %6.2f s   duration %6.2f s\n', ...
            names{k}, s.phase_entry_s(k), s.phase_dur_s(k));
    else
        fprintf('  %-10s (not reached)\n', names{k});
    end
end
L('Total mission time', s.mission_time_s, 's', '%8.2f');
hr();
fprintf('`stats` struct saved to base workspace.\n\n');

% ======================== local helpers ========================

function [t, Y] = ds_sig(out, name)
    o = out.(name); t = o.Time(:); d = o.Data;
    if ndims(d) == 3,                        d = permute(d, [3 1 2]); end
    if size(d,1) ~= numel(t) && size(d,2) == numel(t), d = d.'; end
    Y = d;
end

function v = ds_getvar(name, default)
    try
        v = evalin('base', name);
    catch
        v = default;
    end
end

function r = ds_rms(x)
    x = x(~isnan(x));
    if isempty(x), r = NaN; else, r = sqrt(mean(x.^2)); end
end

function v = ds_pick(t, i)
    if isempty(i) || isnan(i) || i < 1 || i > numel(t), v = NaN; else, v = t(i); end
end

function j = ds_nextidx(i, n)
    if isempty(i), j = 1; else, j = min(i+1, n); end
end
