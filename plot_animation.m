% plot_animation.m
% Animated four-pane flight dashboard for the variable-tilt quadcopter.
% Run after a simulation so that `out` is in the workspace.
%
% Layout:
%   Top-left     — world-frame 3D view with ground plane and flight trail
%   Top-right    — drone close-up (axis-free isometric, tilt clearly visible)
%   Bottom-left  — 2x2 tilt gauges for each rotor (M1..M4)
%   Bottom-right — top-down trajectory map with heading and velocity arrows

% ---- Video export ----
% Set save_video = true to write an MP4 instead of playing in real time.
save_video     = false;
video_file     = 'flight_animation.mp4';
target_frames  = 900;
playback_speed = 1.0;

% ---- Pull logged signals ----
position = squeeze(out.Xe.Data);      if size(position,1)==3, position = position.'; end
velocity = squeeze(out.Ve.Data);      if size(velocity,1)==3, velocity = velocity.'; end
euler    = squeeze(out.Euler.Data);   if size(euler,1)==3,    euler    = euler.';    end
T        = squeeze(out.Tlog.Data);    if size(T,1)==4,        T        = T.';        end
beta     = squeeze(out.BetaLog.Data); if size(beta,1)==4,     beta     = beta.';     end
t        = out.Xe.Time;
tT       = out.Tlog.Time;

% Resample actuator data onto the position time vector
T_anim    = interp1(tT, T,    t, 'previous', 'extrap');
beta_anim = interp1(tT, beta, t, 'previous', 'extrap');

% Clamp altitude so small sim dips below zero don't show underground
position(:,3) = max(position(:,3), 0);

% ---- Constants (keep in sync with build_model.m) ----
arm       = 0.225;
T_max     = 15;
beta_max  = deg2rad(60);
disc_r    = 0.12;
N_disc    = 32;
th_disc   = linspace(0, 2*pi, N_disc + 1);

% Rotor positions in the body frame
rotor_b = arm * [ 1  1 0;
                 -1  1 0;
                 -1 -1 0;
                  1 -1 0].';

% Per-motor colour scheme and labels
mc = [0.90 0.15 0.10;
      0.20 0.50 0.90;
      0.15 0.72 0.25;
      0.95 0.65 0.05];
motor_names = {'M1 FL','M2 RL','M3 RR','M4 FR'};

% Octagon body outline
n_oct  = 8;
th_oct = (0:n_oct-1)' * 2*pi/n_oct + pi/n_oct;
oct_r  = arm * 0.38;
body_b = [oct_r*cos(th_oct), oct_r*sin(th_oct), zeros(n_oct,1)].';

% Try to pick up mission params for axis limits
try
    target_xy_plot = evalin('base','target_xy');
    target_xy_plot = target_xy_plot(:).';
catch
    target_xy_plot = position(end,1:2);
end
try
    h_cruise_plot = evalin('base','h_cruise');
catch
    h_cruise_plot = max(position(:,3));
end

% =====================================================================
%  Figure layout: 4x4 tiled grid — four 2x2 quadrants
% =====================================================================
fig = figure('Name','Tilt-Quad Flight Dashboard','Color','w', ...
             'Position',[40 40 1600 900]);
tl = tiledlayout(fig, 4, 4, 'TileSpacing','compact', 'Padding','compact');

% =====================================================================
%  Pane A: World-frame 3D view (top-left 2x2)
% =====================================================================
ax3d = nexttile(tl, 1, [2 2]);
hold(ax3d,'on'); grid(ax3d,'on'); axis(ax3d,'equal');
xlabel(ax3d,'X (m)'); ylabel(ax3d,'Y (m)'); zlabel(ax3d,'Alt (m)');
view(ax3d, 35, 20);
ax3d.Color     = [0.85 0.92 1.00];
ax3d.GridColor = [0.65 0.65 0.65];
ax3d.GridAlpha = 0.4;
box(ax3d,'on');
title(ax3d,'World view with ground');
disableDefaultInteractivity(ax3d);
ax3d.Interactions = [];
ax3d.Toolbar.Visible = 'off';

world_pad = 3.0;
world_xl = [min([position(:,1); target_xy_plot(1)])-world_pad, ...
            max([position(:,1); target_xy_plot(1)])+world_pad];
world_yl = [min([position(:,2); target_xy_plot(2)])-world_pad, ...
            max([position(:,2); target_xy_plot(2)])+world_pad];
world_zl = [0, max([max(position(:,3))+1, h_cruise_plot+1, 2])];
xlim(ax3d, world_xl); ylim(ax3d, world_yl); zlim(ax3d, world_zl);

% Checkered ground plane
n_tiles = 14;
gx = linspace(world_xl(1), world_xl(2), n_tiles+1);
gy = linspace(world_yl(1), world_yl(2), n_tiles+1);
grass_a = [0.55 0.72 0.45];
grass_b = [0.45 0.62 0.38];
for ii = 1:n_tiles
    for jj = 1:n_tiles
        if mod(ii+jj,2)==0, cc = grass_a; else, cc = grass_b; end
        patch(ax3d, ...
            [gx(ii) gx(ii+1) gx(ii+1) gx(ii)], ...
            [gy(jj) gy(jj) gy(jj+1) gy(jj+1)], ...
            [0 0 0 0], cc, 'EdgeColor','none', 'FaceAlpha',1.0);
    end
end
for ii = 1:numel(gx)
    plot3(ax3d,[gx(ii) gx(ii)], world_yl, [0 0], '-', 'Color',[0.35 0.45 0.30],'LineWidth',0.4);
end
for jj = 1:numel(gy)
    plot3(ax3d, world_xl, [gy(jj) gy(jj)], [0 0], '-', 'Color',[0.35 0.45 0.30],'LineWidth',0.4);
end

% Target flag
plot3(ax3d, target_xy_plot(1)*[1 1], target_xy_plot(2)*[1 1], [0 1.2], ...
    '-','Color',[0.55 0.35 0.05],'LineWidth',2.0);
plot3(ax3d, target_xy_plot(1), target_xy_plot(2), 1.2, 'p', ...
    'MarkerSize',13,'MarkerFaceColor',[0.95 0.75 0.10],'MarkerEdgeColor',[0.4 0.3 0]);
plot3(ax3d, target_xy_plot(1), target_xy_plot(2), 0.01, 'o', ...
    'MarkerSize',7,'MarkerFaceColor',[0.95 0.75 0.10],'MarkerEdgeColor',[0.4 0.3 0]);
plot3(ax3d, position(1,1), position(1,2), 0.01, 's', ...
    'MarkerSize',7,'MarkerFaceColor','k','MarkerEdgeColor','k');

nan8  = nan(1,8);
nan_d = nan(1,N_disc+1);

% Animated handles for the world pane
h_trail3 = plot3(ax3d, NaN, NaN, NaN, '-', 'Color',[0.10 0.45 0.85],'LineWidth',1.2);
h_body   = patch(ax3d, nan8, nan8, nan8, [0.92 0.92 0.92], ...
    'EdgeColor',[0.4 0.4 0.4],'LineWidth',1.4,'FaceAlpha',0.92);
h_arm    = gobjects(4,1);
h_disc   = gobjects(4,1);
h_blade  = gobjects(4,2);
h_thrust = gobjects(4,1);
for j = 1:4
    col_j    = mc(j,:);
    h_arm(j) = plot3(ax3d, [NaN NaN],[NaN NaN],[NaN NaN], '-', 'Color',col_j,'LineWidth',3.5);
    h_disc(j)= fill3(ax3d, nan_d, nan_d, nan_d, col_j, 'FaceAlpha',0.40,'EdgeColor',col_j,'LineWidth',1.4);
    for blade = 1:2
        h_blade(j,blade) = plot3(ax3d, [NaN NaN],[NaN NaN],[NaN NaN], '-', 'Color',col_j*0.7,'LineWidth',2.4);
    end
    h_thrust(j) = quiver3(ax3d,0,0,0,0,0,0,0,'Color',col_j,'LineWidth',2.6,'MaxHeadSize',0.6);
end
triad_len = 0.25;
h_fwd     = quiver3(ax3d,0,0,0,0,0,0,0,'Color',[0.1 0.1 0.1],'LineWidth',1.6,'MaxHeadSize',0.8);
h_fwd_lbl = text(ax3d,0,0,0,'x','Color',[0.1 0.1 0.1],'FontSize',8,'FontWeight','bold');
h_shadow  = patch(ax3d, nan(1,16), nan(1,16), zeros(1,16), [0.15 0.20 0.12], ...
    'EdgeColor','none','FaceAlpha',0.35);
shadow_b  = 1.6 * body_b;

% =====================================================================
%  Pane B: Close-up isometric view (top-right 2x2)
%  No axes, no grid — just the drone geometry so the tilt is very clear.
% =====================================================================
ax3d_close = nexttile(tl, 3, [2 2]);
hold(ax3d_close,'on'); axis(ax3d_close,'equal'); axis(ax3d_close,'off');
view(ax3d_close, 45, 20);
ax3d_close.Color = 'w';
disableDefaultInteractivity(ax3d_close);
ax3d_close.Interactions = [];
ax3d_close.Toolbar.Visible = 'off';

h_body_c   = patch(ax3d_close, nan8, nan8, nan8, [0.92 0.92 0.92], ...
    'EdgeColor',[0.4 0.4 0.4],'LineWidth',1.4,'FaceAlpha',0.92);
h_arm_c    = gobjects(4,1);
h_disc_c   = gobjects(4,1);
h_blade_c  = gobjects(4,2);
h_thrust_c = gobjects(4,1);
for j = 1:4
    col_j      = mc(j,:);
    h_arm_c(j) = plot3(ax3d_close,[NaN NaN],[NaN NaN],[NaN NaN],'-','Color',col_j,'LineWidth',3.5);
    h_disc_c(j)= fill3(ax3d_close,nan_d,nan_d,nan_d,col_j,'FaceAlpha',0.40,'EdgeColor',col_j,'LineWidth',1.4);
    for blade = 1:2
        h_blade_c(j,blade) = plot3(ax3d_close,[NaN NaN],[NaN NaN],[NaN NaN],'-','Color',col_j*0.7,'LineWidth',2.4);
    end
    h_thrust_c(j) = quiver3(ax3d_close,0,0,0,0,0,0,0,'Color',col_j,'LineWidth',2.6,'MaxHeadSize',0.6);
end
h_fwd_c = quiver3(ax3d_close,0,0,0,0,0,0,0,'Color',[0.1 0.1 0.1],'LineWidth',1.6,'MaxHeadSize',0.8);

% =====================================================================
%  Pane C: Tilt gauges 2x2 (bottom-left)
% =====================================================================
ax_tilt    = gobjects(4,1);
tilt_tiles = [9 10 13 14];
for j = 1:4
    ax_tilt(j) = nexttile(tl, tilt_tiles(j), [1 1]);
    hold(ax_tilt(j),'on'); axis(ax_tilt(j),'equal'); axis(ax_tilt(j),'off');
    xlim(ax_tilt(j), [-1.2 1.2]);
    ylim(ax_tilt(j), [-0.3 1.25]);
end

arc_th       = linspace(-beta_max, beta_max, 60);
h_tilt_vec   = gobjects(4,1);
h_tilt_title = gobjects(4,1);
for j = 1:4
    col_j = mc(j,:);
    plot(ax_tilt(j), sin(arc_th), cos(arc_th), '-', 'Color',[0.55 0.55 0.55],'LineWidth',1.0);
    for ang = deg2rad([-60 -30 0 30 60])
        x0 = 0.92*sin(ang); y0 = 0.92*cos(ang);
        x1 = 1.05*sin(ang); y1 = 1.05*cos(ang);
        plot(ax_tilt(j), [x0 x1], [y0 y1], '-', 'Color',[0.4 0.4 0.4],'LineWidth',1.0);
        text(ax_tilt(j), 1.18*sin(ang), 1.18*cos(ang), ...
            sprintf('%d', round(rad2deg(ang))), ...
            'HorizontalAlignment','center','FontSize',7,'Color',[0.4 0.4 0.4]);
    end
    plot(ax_tilt(j), 0, 0, 'o', 'MarkerSize',5, ...
        'MarkerFaceColor',[0.3 0.3 0.3],'MarkerEdgeColor',[0.3 0.3 0.3]);
    plot(ax_tilt(j), [0 0], [0 1], ':', 'Color',[0.7 0.7 0.7]);
    h_tilt_vec(j)   = plot(ax_tilt(j), [0 0], [0 0], '-', 'Color',col_j,'LineWidth',5);
    h_tilt_title(j) = title(ax_tilt(j), ...
        sprintf('%s   \\beta = --   T = --', motor_names{j}), ...
        'Color',col_j,'FontSize',10,'FontWeight','bold');
end

% =====================================================================
%  Pane D: Top-down trajectory map (bottom-right 2x2)
% =====================================================================
ax_map = nexttile(tl, 11, [2 2]);
hold(ax_map,'on'); grid(ax_map,'on'); axis(ax_map,'equal');
xlabel(ax_map,'X (m)'); ylabel(ax_map,'Y (m)');
title(ax_map,'Top-down trajectory');
ax_map.Color = [0.97 0.97 0.97];
disableDefaultInteractivity(ax_map);
ax_map.Toolbar.Visible = 'off';

plot(ax_map, position(:,1), position(:,2), '-', 'Color',[0.7 0.7 0.7],'LineWidth',1.0);
plot(ax_map, position(1,1), position(1,2), 'ks', 'MarkerFaceColor','k','MarkerSize',6);
plot(ax_map, target_xy_plot(1), target_xy_plot(2), 'p', ...
    'MarkerSize',13,'MarkerFaceColor',[0.95 0.75 0.10],'MarkerEdgeColor',[0.4 0.3 0]);
text(ax_map, target_xy_plot(1), target_xy_plot(2), '  target', 'FontSize',8,'Color',[0.3 0.25 0]);
map_pad = max(2, 0.1*max(max(position(:,1:2),[],1) - min(position(:,1:2),[],1)));
map_xl  = [min([position(:,1); target_xy_plot(1)])-map_pad, ...
           max([position(:,1); target_xy_plot(1)])+map_pad];
map_yl  = [min([position(:,2); target_xy_plot(2)])-map_pad, ...
           max([position(:,2); target_xy_plot(2)])+map_pad];
xlim(ax_map, map_xl); ylim(ax_map, map_yl);

h_trail   = plot(ax_map, NaN, NaN, '-', 'Color',[0.10 0.45 0.85],'LineWidth',2.0);
h_drone2d = plot(ax_map, NaN, NaN, 'o', 'MarkerSize',8, ...
    'MarkerFaceColor',[0.10 0.45 0.85],'MarkerEdgeColor','k');
h_head2d  = quiver(ax_map,0,0,0,0,0,'Color',[0.10 0.45 0.85],'LineWidth',1.8,'MaxHeadSize',1.2);
h_vel2d   = quiver(ax_map,0,0,0,0,0,'Color',[0.7 0 0.8],'LineWidth',1.6,'MaxHeadSize',1.2);

h_sgt = sgtitle(tl, '', 'FontSize',12,'FontWeight','normal');

% ---- Frame schedule ----
total_time  = t(end) - t(1);
step        = max(1, round((length(t)-1) / target_frames));
n_frames    = numel(1:step:length(t));
write_fps   = max(1, (n_frames / total_time) * playback_speed);
frame_dt    = total_time / n_frames / playback_speed;

if save_video
    vid = VideoWriter(video_file, 'MPEG-4');
    vid.FrameRate = write_fps;
    open(vid);
end

trail_window = 2.0;
close_pad    = 0.9;
loop_t0      = tic;
frame_idx    = 0;

% ---- Main animation loop ----
for fi = 1:step:length(t)
    frame_idx = frame_idx + 1;
    x = position(fi,1); y = position(fi,2); z = max(position(fi,3), 0);
    phi = euler(fi,1); theta = euler(fi,2); psi = euler(fi,3);

    R = rotz(rad2deg(psi)) * roty(rad2deg(theta)) * rotx(rad2deg(phi));

    % Body octagon patch
    body_e = R * body_b;
    bx = x+body_e(1,:); by = y+body_e(2,:); bz = max(z+body_e(3,:),0);
    set(h_body,   'XData',bx,'YData',by,'ZData',bz);
    set(h_body_c, 'XData',bx,'YData',by,'ZData',bz);

    % Ground shadow fades with altitude
    sh_e         = rotz(rad2deg(psi)) * shadow_b;
    shadow_alpha = max(0.05, 0.35 * exp(-z/4));
    set(h_shadow, 'XData',x+sh_e(1,:), 'YData',y+sh_e(2,:), ...
        'ZData',0.005*ones(1,size(sh_e,2)), 'FaceAlpha',shadow_alpha);

    prop_spin = mod(fi * 28, 360);

    for j = 1:4
        rj_b = rotor_b(:,j);
        rj_e = R * rj_b;
        cx = x+rj_e(1); cy = y+rj_e(2); cz = max(z+rj_e(3), 0);

        set(h_arm(j),   'XData',[x cx],'YData',[y cy],'ZData',[z cz]);
        set(h_arm_c(j), 'XData',[x cx],'YData',[y cy],'ZData',[z cz]);

        % Rotor disc — tilted by beta around the y-axis of the rotor's local frame
        bj   = beta_anim(fi,j);
        v1_b = [cos(bj); 0; -sin(bj)];
        v2_b = [0; 1; 0];
        v1_e = R * v1_b;
        v2_e = R * v2_b;
        dx_ = disc_r*(cos(th_disc).*v1_e(1) + sin(th_disc).*v2_e(1));
        dy_ = disc_r*(cos(th_disc).*v1_e(2) + sin(th_disc).*v2_e(2));
        dz_ = disc_r*(cos(th_disc).*v1_e(3) + sin(th_disc).*v2_e(3));
        set(h_disc(j),   'XData',cx+dx_,'YData',cy+dy_,'ZData',cz+dz_);
        set(h_disc_c(j), 'XData',cx+dx_,'YData',cy+dy_,'ZData',cz+dz_);

        % Spinning blades (opposite rotation directions for alternate rotors)
        spin_dir = (-1)^(j+1);
        pa = deg2rad(prop_spin * spin_dir);
        for blade = 1:2
            ang = pa + (blade-1)*pi/2;
            bp1 = disc_r*0.95*(cos(ang)*v1_e + sin(ang)*v2_e);
            bp2 = disc_r*0.95*(-cos(ang)*v1_e - sin(ang)*v2_e);
            set(h_blade(j,blade),   'XData',cx+[bp1(1) bp2(1)],'YData',cy+[bp1(2) bp2(2)],'ZData',cz+[bp1(3) bp2(3)]);
            set(h_blade_c(j,blade), 'XData',cx+[bp1(1) bp2(1)],'YData',cy+[bp1(2) bp2(2)],'ZData',cz+[bp1(3) bp2(3)]);
        end

        % Thrust arrow scaled by rotor thrust magnitude
        Tj       = T_anim(fi,j);
        thrust_b = [sin(bj); 0; cos(bj)];
        thrust_e = R * thrust_b;
        arrow_len = Tj * 0.075;
        set(h_thrust(j),   'XData',cx,'YData',cy,'ZData',cz, ...
            'UData',thrust_e(1)*arrow_len,'VData',thrust_e(2)*arrow_len,'WData',thrust_e(3)*arrow_len);
        set(h_thrust_c(j), 'XData',cx,'YData',cy,'ZData',cz, ...
            'UData',thrust_e(1)*arrow_len,'VData',thrust_e(2)*arrow_len,'WData',thrust_e(3)*arrow_len);

        % Tilt gauge card
        len_j = 0.25 + 0.75*min(1, max(0, Tj/T_max));
        set(h_tilt_vec(j), 'XData',[0 len_j*sin(bj)],'YData',[0 len_j*cos(bj)]);
        set(h_tilt_title(j), 'String', ...
            sprintf('%s   \\beta = %+5.1f\\circ   T = %4.1f N', motor_names{j}, rad2deg(bj), Tj));
    end

    % Forward body-x indicator
    fwd_e = R * [triad_len; 0; 0];
    set(h_fwd,   'XData',x,'YData',y,'ZData',z,'UData',fwd_e(1),'VData',fwd_e(2),'WData',fwd_e(3));
    set(h_fwd_lbl,'Position',[x+fwd_e(1)*1.4, y+fwd_e(2)*1.4, z+fwd_e(3)*1.4]);
    set(h_fwd_c, 'XData',x,'YData',y,'ZData',z,'UData',fwd_e(1),'VData',fwd_e(2),'WData',fwd_e(3));

    % World pane trail
    set(h_trail3, 'XData',position(1:fi,1),'YData',position(1:fi,2),'ZData',max(position(1:fi,3),0));

    % Close-up pane: keep drone centred
    xlim(ax3d_close, [x-close_pad, x+close_pad]);
    ylim(ax3d_close, [y-close_pad, y+close_pad]);
    zlim(ax3d_close, [max(0,z-close_pad), z+close_pad]);

    % Top-down map
    trail_mask = (t >= t(fi) - trail_window) & (t <= t(fi));
    set(h_trail,   'XData',position(trail_mask,1),'YData',position(trail_mask,2));
    set(h_drone2d, 'XData',x,'YData',y);
    head_len = 0.08 * max(map_xl(2)-map_xl(1), map_yl(2)-map_yl(1));
    set(h_head2d,  'XData',x,'YData',y,'UData',head_len*cos(psi),'VData',head_len*sin(psi));
    set(h_vel2d,   'XData',x,'YData',y,'UData',velocity(fi,1)*0.3,'VData',velocity(fi,2)*0.3);

    Vmag     = hypot(velocity(fi,1), velocity(fi,2));
    dist_left= hypot(target_xy_plot(1)-x, target_xy_plot(2)-y);
    set(h_sgt,'String', sprintf('t = %5.2f s   |   alt = %5.2f m   |   |V_{xy}| = %4.1f m/s   |   to-target = %5.2f m', ...
        t(fi), z, Vmag, dist_left));

    if save_video
        drawnow;
        writeVideo(vid, getframe(fig));
    else
        drawnow limitrate;
        target_elapsed = frame_idx * frame_dt;
        actual_elapsed = toc(loop_t0);
        if target_elapsed > actual_elapsed
            pause(target_elapsed - actual_elapsed);
        end
    end
end

if save_video
    close(vid);
    fprintf('Saved: %s\n', video_file);
end
