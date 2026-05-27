% plot_metrics.m
% Static 3x3 metrics dashboard for the variable-tilt quadcopter.
% Run after a simulation so that `out` is in the workspace.
% All chart definitions are in figure_defs.m so they're shared with
% any other plotting scripts.

assert(exist('out', 'var') == 1, 'plot_metrics: run a simulation first (no `out` in workspace).');

[P, meta] = figure_defs(out);

fig = figure('Name', 'Tilt-quad metrics', 'Color', 'w', 'Position', [60 50 1300 850]);
set(fig, 'defaultAxesFontName', 'Arial', 'defaultTextFontName', 'Arial', ...
         'defaultLegendFontName', 'Arial');
tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, meta.suptitle, 'Color', 'k');

axs = gobjects(numel(P), 1);
for i = 1:numel(P)
    ax = nexttile;
    P(i).draw(ax);
    title(ax, P(i).title, 'Color', 'k');
    axs(i) = ax;
end

% Link the time axes so panning one panel pans all of them
linkaxes(axs(meta.timepanels), 'x');
xlim(axs(meta.timepanels(1)), meta.tspan);
