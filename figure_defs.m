function [P, meta] = figure_defs(out)
% figure_defs  Chart definitions for the variable-tilt quadcopter metrics.
%
% Returns a struct array P where each entry has:
%   P(i).name   - short id used as a filename-safe label
%   P(i).title  - panel title shown on the figure
%   P(i).draw   - function handle @(ax) that draws the chart into ax
%
% Also returns meta with a mission title string, the indices of time-linked
% panels, and the time axis range.
%
% Used by plot_metrics.m for the tiled dashboard.

assert(nargin == 1 && ~isempty(out), 'figure_defs: pass a sim output struct `out`.');

% Grab params from the workspace, fall back to sensible defaults if missing
h_cruise = dg_getvar('h_cruise',  5);
v_cruise = dg_getvar('v_cruise',  8);
T_max    = dg_getvar('T_max',    15);
beta_max = dg_getvar('beta_max', deg2rad(60));
target   = dg_getvar('target_xy', [25 10]);

% Pull logged signals
[t ,Xe] = dg_sig(out,'Xe');     alt=Xe(:,3); posN=Xe(:,1); posE=Xe(:,2);
[~ ,Ve] = dg_sig(out,'Ve');     gspd=hypot(Ve(:,1),Ve(:,2));
[~ ,Vb] = dg_sig(out,'Vb');     vbx=Vb(:,1);
[~ ,Eu] = dg_sig(out,'Euler');  phi=Eu(:,1); th=Eu(:,2); psi=Eu(:,3);
[~ ,W ] = dg_sig(out,'PQR');
[tT,T ] = dg_sig(out,'Tlog');
[tB,B ] = dg_sig(out,'BetaLog');
[tH,Hd] = dg_sig(out,'HDesLog');
[tV,Vd] = dg_sig(out,'VxDesLog');
[tP,Pd] = dg_sig(out,'PsiDesLog');
[tS,St] = dg_sig(out,'StateLog'); St=round(St);
[tF,Fx] = dg_sig(out,'FxDes');
[~ ,Fz] = dg_sig(out,'FzDes');

% Interpolate setpoints onto the main time vector so error plots line up
Hd_i = interp1(tH, Hd, t, 'linear', 'extrap');
Pd_i = interp1(tP, Pd, t, 'linear', 'extrap');
Vd_i = interp1(tV, Vd, t, 'linear', 'extrap');

pal   = dg_pal();
co    = [pal.deepblue; pal.brick; pal.olive; pal.slate];
names = {'TO','ALIGN','CRUISE','HOVER','DESC','LAND'};
di    = find(diff(St) ~= 0);
trT   = tS(di+1);  trS = St(di+1);

% Register each panel — the function handles capture the variables above
P = struct('name',{},'title',{},'draw',{});
P = add(P, '01_altitude',    'Altitude tracking',              @(ax) p_alt(ax,t,alt,Hd_i,trT,trS,names));
P = add(P, '02_speed',       'Speed tracking',                 @(ax) p_speed(ax,t,gspd,vbx,Vd_i,v_cruise));
P = add(P, '03_ground_track','Ground track',                   @(ax) p_track(ax,posN,posE,target));
P = add(P, '04_attitude',    'Attitude (roll / pitch / yaw)',  @(ax) p_att(ax,t,phi,th,psi,Pd_i));
P = add(P, '05_body_rates',  'Body angular rates',             @(ax) p_rates(ax,t,W));
P = add(P, '06_thrust',      'Per-rotor thrust',               @(ax) p_thrust(ax,tT,T,T_max,co));
P = add(P, '07_tilt',        'Per-rotor tilt',                 @(ax) p_tilt(ax,tB,B,beta_max,co));
P = add(P, '08_mission_state','Mission state machine',         @(ax) p_state(ax,tS,St,names));
P = add(P, '09_force_demands','Controller force demands',      @(ax) p_force(ax,tF,Fx,Fz));

meta.suptitle    = sprintf(['Variable-tilt quad — mission to [%g %g] m, ' ...
    'h_{cruise}=%g m, v_{cruise}=%g m/s'], target(1), target(2), h_cruise, v_cruise);
meta.timepanels  = [1 2 4 5 6 7 8 9];   % all panels except the ground track
meta.tspan       = [t(1) t(end)];
end

% ======================== panel drawers ========================

function p_alt(ax,t,alt,Hd,trT,trS,names)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on');
    plot(ax, t, alt, 'Color', p.deepblue, 'LineWidth', 1.4);
    plot(ax, t, Hd,  'Color', p.gray, 'LineStyle', '--', 'LineWidth', 1.2);
    for k = 1:numel(trT)
        xline(ax, trT(k), ':', names{min(max(trS(k),1),numel(names))}, ...
            'Color', [0.6 0.6 0.6], 'LabelVerticalAlignment', 'bottom', ...
            'FontSize', 8, 'HandleVisibility', 'off');
    end
    ylabel(ax,'Altitude (m)'); xlabel(ax,'t (s)');
    legend(ax, {'altitude','h_{des}'}, 'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_speed(ax,t,~,vbx,Vd,v_cruise)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on');
    plot(ax, t, vbx, 'Color', p.slate, 'LineWidth', 1.3);
    plot(ax, t, Vd,  'Color', p.gray,  'LineStyle', '--', 'LineWidth', 1.2);
    yline(ax, v_cruise, ':', 'v_{cruise}', 'Color', p.gray, 'HandleVisibility', 'off');
    ylabel(ax,'Speed (m/s)'); xlabel(ax,'t (s)');
    legend(ax, {'V_{b,xy} forward','V_{xy,des}'}, 'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_track(ax,posN,posE,target)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    plot(ax, posE, posN, 'Color', p.deepblue, 'LineWidth', 1.4);
    plot(ax, posE(1),   posN(1),   'o', 'Color', p.olive, 'MarkerFaceColor', p.olive,   'MarkerSize', 8);
    plot(ax, target(2), target(1), 'p', 'Color', p.brick, 'MarkerFaceColor', p.brick,   'MarkerSize', 14);
    plot(ax, posE(end), posN(end), 's', 'Color', p.slate, 'MarkerFaceColor', p.slate,   'MarkerSize', 8);
    xlabel(ax,'East (m)'); ylabel(ax,'North (m)');
    legend(ax, {'path','start','target','end'}, 'Location','southoutside','Orientation','horizontal');
    allE = [posE(:); target(2)];
    allN = [posN(:); target(1)];
    pad  = max([max(allE)-min(allE), max(allN)-min(allN), 1]) * 0.18;
    xlim(ax, [min(allE)-pad, max(allE)+pad]);
    ylim(ax, [min(allN)-pad, max(allN)+pad]);
    dg_axstyle(ax);
end

function p_att(ax,t,phi,th,psi,Pd)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on');
    plot(ax, t, rad2deg(phi),          'Color', p.deepblue, 'LineWidth', 1.2);
    plot(ax, t, rad2deg(th),           'Color', p.brick,    'LineWidth', 1.2);
    plot(ax, t, rad2deg(psi),          'Color', p.olive,    'LineWidth', 1.4);
    plot(ax, t, rad2deg(unwrap(Pd)),   'Color', p.gray,     'LineStyle', '--', 'LineWidth', 1.0);
    ylabel(ax,'Angle (deg)'); xlabel(ax,'t (s)');
    legend(ax, {'roll \phi','pitch \theta','yaw \psi','\psi_{des}'}, ...
        'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_rates(ax,t,W)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on');
    plot(ax, t, rad2deg(W(:,1)), 'Color', p.deepblue, 'LineWidth', 1.1);
    plot(ax, t, rad2deg(W(:,2)), 'Color', p.brick,    'LineWidth', 1.1);
    plot(ax, t, rad2deg(W(:,3)), 'Color', p.olive,    'LineWidth', 1.1);
    ylabel(ax,'Body rate (deg/s)'); xlabel(ax,'t (s)');
    legend(ax, {'p','q','r'}, 'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_thrust(ax,tT,T,T_max,co)
    hold(ax,'on'); grid(ax,'on');
    for i = 1:size(T,2)
        plot(ax, tT, T(:,i), 'LineWidth', 1.2, 'Color', co(i,:));
    end
    yline(ax, T_max, '--', 'T_{max}', 'Color', dg_pal().gray, 'HandleVisibility', 'off');
    ylabel(ax,'Thrust T_i (N)'); xlabel(ax,'t (s)');
    legend(ax, {'T_1','T_2','T_3','T_4'}, 'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_tilt(ax,tB,B,beta_max,co)
    hold(ax,'on'); grid(ax,'on');
    for i = 1:size(B,2)
        plot(ax, tB, rad2deg(B(:,i)), 'LineWidth', 1.2, 'Color', co(i,:));
    end
    gry = dg_pal().gray;
    yline(ax,  rad2deg(beta_max), '--', '+\beta_{max}', 'Color', gry, 'HandleVisibility', 'off');
    yline(ax, -rad2deg(beta_max), '--', '-\beta_{max}', 'Color', gry, 'HandleVisibility', 'off');
    ylabel(ax,'Tilt \beta_i (deg)'); xlabel(ax,'t (s)');
    legend(ax, {'\beta_1','\beta_2','\beta_3','\beta_4'}, ...
        'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

function p_state(ax,tS,St,names)
    grid(ax,'on');
    stairs(ax, tS, St, 'Color', dg_pal().deepblue, 'LineWidth', 1.6);
    ylim(ax,[0.5 6.5]); yticks(ax,1:6); yticklabels(ax,names);
    ylabel(ax,'State'); xlabel(ax,'t (s)');
    dg_axstyle(ax);
end

function p_force(ax,tF,Fx,Fz)
    p = dg_pal();
    hold(ax,'on'); grid(ax,'on');
    plot(ax, tF, Fx, 'Color', p.deepblue, 'LineWidth', 1.2);
    plot(ax, tF, Fz, 'Color', p.brick,    'LineWidth', 1.2);
    ylabel(ax,'Demand (N)'); xlabel(ax,'t (s)');
    legend(ax, {'F_{x,des}','F_{z,des}'}, 'Location','southoutside','Orientation','horizontal');
    dg_axstyle(ax);
end

% ======================== helpers ========================

function P = add(P, nm, ti, fn)
    P(end+1) = struct('name', nm, 'title', ti, 'draw', fn);
end

function p = dg_pal()
% Muted, print-friendly colour palette. Same colours used throughout so the
% report figures look consistent even across different chart types.
    p.deepblue = [0.122 0.306 0.475];
    p.brick    = [0.620 0.239 0.133];
    p.olive    = [0.239 0.420 0.208];
    p.slate    = [0.290 0.290 0.416];
    p.gray     = [0.353 0.353 0.353];
end

function dg_axstyle(ax)
% Apply consistent axis cosmetics to every panel.
    set(ax, 'FontName','Arial', 'FontSize',10, 'LineWidth',0.75, ...
        'Color','w', 'Box','on', 'Layer','top', 'TickDir','out', ...
        'XColor',[0.15 0.15 0.15], 'YColor',[0.15 0.15 0.15], ...
        'GridColor',[0.85 0.85 0.85], 'GridAlpha',0.6, ...
        'MinorGridLineStyle','none');
    ax.Title.Color = 'k';
    lg = get(ax, 'Legend');
    if ~isempty(lg), set(lg, 'Box','off', 'FontSize',9, 'TextColor','k'); end
end

function [t, Y] = dg_sig(out, name)
% Extract a time-series signal from the sim output struct into a plain matrix.
    o = out.(name); t = o.Time(:); d = o.Data;
    if ndims(d) == 3,                        d = permute(d, [3 1 2]); end
    if size(d,1) ~= numel(t) && size(d,2) == numel(t), d = d.'; end
    Y = d;
end

function v = dg_getvar(name, default)
% Safe workspace variable lookup — returns `default` if the variable doesn't exist.
    try
        v = evalin('base', name);
    catch
        v = default;
    end
end
