function animate_entry_attitude(t, x, R, stride)
% Interactive trajectory viewer with:
% - capsule position
% - velocity-direction vector
% - capsule-axis vector
% - slider to scrub forward/backward in time
%
% In this 2D interpretation:
% - gamma is the velocity-direction angle relative to local horizontal
% - alpha is the angle between body axis and freestream
% - the body-axis angle is approximated as theta_body = gamma + alpha

if nargin < 4
    stride = 5; % default play step in animation samples; higher = faster but less smooth
end

speedup = 20; % play the trajectory 20 times faster than physical time
fps = 30;     % target visual refresh rate for the viewer
playback_dt = 1 / fps;

t_anim = (t(1):speedup*playback_dt:t(end)).';
if t_anim(end) < t(end)
    t_anim = [t_anim; t(end)];
end

x_anim = interp1(t, x, t_anim, 'pchip');
mach_anim = interp1(t, R.Mach, t_anim, 'pchip');
alpha_anim = interp1(t, R.alpha_deg, t_anim, 'pchip');

s_km = x_anim(:,4) / 1000;
h_km = x_anim(:,3) / 1000;

fig = figure('Color', 'w', 'Position', [150 120 900 680]);
hold on
grid on
box on
plot(x(:,4)/1000, x(:,3)/1000, 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0)
xlabel('Downrange [km]')
ylabel('Altitude [km]')
title('Trajectory, Velocity Vector and Capsule Axis', 'Color', 'k')

traj_pt = plot(s_km(1), h_km(1), 'o', 'MarkerFaceColor', [0 0.45 0.74], ...
    'MarkerEdgeColor', 'none', 'MarkerSize', 7);

vel_vec = quiver(s_km(1), h_km(1), 0, 0, 0, 'Color', [0 0.45 0.74], ...
    'LineWidth', 1.8, 'MaxHeadSize', 2);
body_vec = quiver(s_km(1), h_km(1), 0, 0, 0, 'Color', [0.85 0.33 0.10], ...
    'LineWidth', 1.8, 'MaxHeadSize', 2);
vel_label = text(s_km(1), h_km(1), 'v', 'Color', [0 0.45 0.74], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom');
body_label = text(s_km(1), h_km(1), 'body', 'Color', [0.85 0.33 0.10], ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'bottom');

txt = text(s_km(1), h_km(1), '', 'VerticalAlignment', 'bottom', ...
    'HorizontalAlignment', 'left', 'Color', 'k');

axis tight
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
    'GridColor', [0.75 0.75 0.75], 'FontName', 'Helvetica')

scale = max(max(h_km) - min(h_km), max(s_km) - min(s_km));
vec_len = 0.08 * max(scale, 1);

uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.12 0.01 0.10 0.035], 'String', 'Time index', ...
    'BackgroundColor', 'w', 'ForegroundColor', 'k');

slider = uicontrol(fig, 'Style', 'slider', 'Units', 'normalized', ...
    'Position', [0.24 0.015 0.52 0.03], ...
    'Min', 1, 'Max', numel(t_anim), 'Value', 1, ...
    'SliderStep', [1/max(numel(t_anim)-1,1), 20/max(numel(t_anim)-1,1)]);

time_label = uicontrol(fig, 'Style', 'text', 'Units', 'normalized', ...
    'Position', [0.78 0.01 0.14 0.035], 'String', '', ...
    'BackgroundColor', 'w', 'ForegroundColor', 'k');

play_btn = uicontrol(fig, 'Style', 'togglebutton', 'Units', 'normalized', ...
    'Position', [0.02 0.01 0.08 0.04], 'String', 'Play', ...
    'BackgroundColor', [0.94 0.94 0.94], 'ForegroundColor', 'k');

    function update_frame(i)
        i = max(1, min(numel(t_anim), round(i)));
        gamma = x_anim(i,2);
        alpha = deg2rad(alpha_anim(i));
        theta_body = gamma + alpha;

        vdir = [cos(gamma), sin(gamma)];
        bdir = [cos(theta_body), sin(theta_body)];

        set(traj_pt, 'XData', s_km(i), 'YData', h_km(i))
        set(vel_vec, 'XData', s_km(i), 'YData', h_km(i), ...
            'UData', vec_len * vdir(1), 'VData', vec_len * vdir(2))
        set(body_vec, 'XData', s_km(i), 'YData', h_km(i), ...
            'UData', vec_len * bdir(1), 'VData', vec_len * bdir(2))
        set(vel_label, 'Position', [s_km(i) + 1.05*vec_len*vdir(1), ...
            h_km(i) + 1.05*vec_len*vdir(2), 0])
        set(body_label, 'Position', [s_km(i) + 1.05*vec_len*bdir(1), ...
            h_km(i) + 1.05*vec_len*bdir(2), 0])
        set(txt, 'Position', [s_km(i), h_km(i), 0], ...
            'String', sprintf('t = %.1f s | Mach = %.1f | alpha = %.1f deg', ...
            t_anim(i), mach_anim(i), alpha_anim(i)))
        set(time_label, 'String', sprintf('t = %.1f s', t_anim(i)))
        set(slider, 'Value', i)
        drawnow
    end

slider.Callback = @(src,~) update_frame(src.Value);
play_btn.Callback = @(src,~) toggle_play(src);

update_frame(1)

    function toggle_play(btn)
        if get(btn, 'Value') == 1
            curr_val = round(get(slider, 'Value'));
            if curr_val >= numel(t_anim)
                curr_val = 1;
            end
            for idx = curr_val:stride:numel(t_anim)
                if ~isvalid(fig) || get(btn, 'Value') == 0
                    break
                end
                update_frame(idx)
                pause(playback_dt)
            end
            if isvalid(btn)
                set(btn, 'Value', 0)
            end
        end
    end
end
