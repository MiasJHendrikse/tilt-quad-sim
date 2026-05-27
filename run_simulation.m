% run_simulation.m
% Main entry point for the variable-tilt quadcopter simulation.
% Run this script to build parameters, simulate, and visualise results.
%
% Steps:
%   1. Load mission parameters (or prompt for them if not set yet).
%   2. Build the Simulink model if the .slx is missing.
%   3. Run the simulation.
%   4. Optionally show the animated dashboard, the metrics charts, or the stats report.
%
% Flip the toggles below to choose what runs after the sim.

% ============== TOGGLES ==============
run_plot    = false;   % animated 4-pane dashboard (plot_animation.m)
run_metrics = true;    % static 3x3 time-history charts
run_stats   = false;   % quantitative performance report in Command Window
% =====================================

mdl = 'tilt_quad_sim';

% Load parameters. Prompts are skipped if the three key vars are already set.
params;

% Build the model from scratch if it isn't there yet
if ~exist([mdl '.slx'], 'file')
    fprintf('Model %s.slx not found — building it now.\n', mdl);
    build_model;
end

% Make sure the model is loaded before calling sim()
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

fprintf('Running simulation: %s ...\n', mdl);
out = sim(mdl);
assignin('base', 'out', out);
fprintf('Simulation complete (%.1f s simulated).\n', sim_time);

if run_plot
    fprintf('Opening animated dashboard ...\n');
    plot_animation;
end

if run_metrics
    fprintf('Plotting metrics ...\n');
    plot_metrics;
end

if run_stats
    fprintf('Computing performance stats ...\n');
    mission_stats;
end

fprintf('Done.\n');
