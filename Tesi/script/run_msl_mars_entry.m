clearvars; clc; close all;

% MSL-like Mars entry model with:
% - aerodynamic coefficients tabulated as functions of Mach and alpha
% - prescribed alpha(M) schedule as a surrogate for active attitude control
% - variable Mars gravity
% - simple thermal model for a PICA-like TPS
% - parachute deployment and stop-altitude events
%
% Every parameter is defined in build_params.m and tagged there as:
%   - source: taken directly from public references
%   - estimate: engineering estimate used because a public mission value
%               is not readily available

% -------------------------------------------------------------------------
% CONFIGURAZIONE VEICOLO / MISSIONE
% Opzioni disponibili:
%   'msl'        -> MSL / Mars 2020 Curiosity/Perseverance (2.8 t, D = 4.5 m)
%   'red_dragon' -> SpaceX Red Dragon Concept (9.7 t, D = 3.7 m)
%   'hiad_human' -> NASA Human Mars Lander (15.0 t, D = 10.0 m)
% -------------------------------------------------------------------------
if ~exist('vehicle_choice', 'var') || isempty(vehicle_choice)
    vehicle_choice = 'msl';
end

switch lower(vehicle_choice)
    case 'red_dragon'
        P = build_params_red_dragon();
    case 'hiad_human'
        P = build_params_hiad_human();
    otherwise
        P = build_params();
end

enable_animation = false;
atm0 = mars_atmosphere(P.init.h0, P);
alpha0 = alpha_schedule(P.init.v0 / atm0.a, P); % initial alpha reference from initial Mach

% -------------------------
% INITIAL CONDITIONS
% -------------------------
x0 = [ ...
    P.init.v0;          % velocity [m/s]
    P.init.gamma0;      % flight-path angle [rad]
    P.init.h0;          % altitude [m]
    P.init.s0;          % downrange [m]
    P.init.T_tps0;      % external TPS lump temperature [K]
    P.init.T_inner0;    % inner wall temperature [K]
    P.init.thickness_tps0;  % remaining TPS thickness [m]
    alpha0;             % actual alpha [deg]
    P.init.CdA_chute0;  % effective chute Cd*A factor [m^2]
    P.init.A_eff0];     % effective aero area [m^2]

% -------------------------
% ENTRY PHASE A
% Descend until the parachute deployment altitude window is reached,
% or until another terminal condition is met.
% -------------------------
opts_entry_a = odeset( ...
    'RelTol', 1e-6, ...
    'AbsTol', 1e-7, ...
    'Events', @(t,x) event_entry_phase_a(t, x, P));

[t1a, x1a, te1a, xe1a, ie1a] = ode45(@(t,x) dynamics_entry(t, x, P), ...
    [0, P.sim.tmax_entry], x0, opts_entry_a);

% -------------------------
% ENTRY PHASE B
% After entering the allowed altitude window, continue until Mach reaches
% the deploy threshold, or until another terminal condition is met.
% -------------------------
t1 = t1a;
x1 = x1a;
te = te1a;
xe = xe1a;
ie = ie1a;

entered_chute_window = ~isempty(ie1a) && ie1a(end) == 1;

if entered_chute_window
    opts_entry_b = odeset( ...
        'RelTol', 1e-6, ...
        'AbsTol', 1e-7, ...
        'Events', @(t,x) event_entry_phase_b(t, x, P));

    [t1b, x1b, te1b, xe1b, ie1b] = ode45(@(t,x) dynamics_entry(t, x, P), ...
        [te1a(end), P.sim.tmax_entry], xe1a(end,:).', opts_entry_b);

    t1 = [t1a; t1b(2:end)];
    x1 = [x1a; x1b(2:end,:)];
    te = te1b;
    xe = xe1b;
    ie = ie1b;
end

% -------------------------
% PARACHUTE PHASE
% -------------------------
t = t1;
x = x1;
phase_info.idx_entry_end = numel(t1); % last stored index still governed by entry dynamics
phase_info.idx_subchute_start = Inf;  % default if the subsonic chute stage is never reached
phase_info.t_supchute = NaN;          % default if the supersonic chute never deploys
phase_info.t_subchute = NaN;          % default if the subsonic chute never deploys

if ~isempty(ie) && ie(end) == 1
    opts_chute_1 = odeset( ...
        'RelTol', 1e-6, ...
        'AbsTol', 1e-7, ...
        'Events', @(t,x) event_chute_stage1(t, x, P));

    [t2a, x2a, te2a, xe2a, ie2a] = ode45(@(t,x) dynamics_chute_stage(t, x, P, 1), ...
        [te(end), P.sim.tmax_total], xe(end,:).', opts_chute_1);

    phase_info.t_supchute = te(end);
    t = [t1; t2a(2:end)];
    x = [x1; x2a(2:end,:)];

    if ~isempty(ie2a) && ie2a(end) == 1
        opts_chute_2 = odeset( ...
            'RelTol', 1e-6, ...
            'AbsTol', 1e-7, ...
            'Events', @(t,x) event_chute_stage2(t, x, P));

        [t2b, x2b, te2b, ~, ie2b] = ode45(@(t,x) dynamics_chute_stage(t, x, P, 2), ...
            [te2a(end), P.sim.tmax_total], xe2a(end,:).', opts_chute_2);

        phase_info.idx_subchute_start = numel(t1) + numel(t2a(2:end)) + 1;
        phase_info.t_subchute = te2a(end);

        t = [t1; t2a(2:end); t2b(2:end)];
        x = [x1; x2a(2:end,:); x2b(2:end,:)];

        if isempty(ie2b)
            phase_info.t_subchute = t2a(end);
        end
    end
end

% -------------------------
% POST-PROCESSING
% -------------------------
R = compute_derived_history(t, x, P, phase_info); % reconstruct diagnostic outputs for plotting

% Peak temperature tracking
[T_tps_peak, idx_tps_peak] = max(x(:,5));
t_tps_peak = t(idx_tps_peak);
h_tps_peak = x(idx_tps_peak, 3);
v_tps_peak = x(idx_tps_peak, 1);

[T_inner_peak, idx_inner_peak] = max(x(:,6));
t_inner_peak = t(idx_inner_peak);
h_inner_peak = x(idx_inner_peak, 3);
v_inner_peak = x(idx_inner_peak, 1);

thickness_consumed = P.init.thickness_tps0 - x(end,7);
[g_peak, idx_g_peak] = max(R.g_load_earth);
[q_peak, idx_q_peak] = max(R.q_conv);

% -------------------------
% PLOTS
% -------------------------
figure('Color','w','Position',[100 80 1250 900]);

subplot(3,5,1)
plot(t, x(:,1), 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('Velocity [m/s]'); grid on

subplot(3,5,2)
plot(t, x(:,3)/1000, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('Altitude [km]'); grid on

subplot(3,5,3)
plot(t, x(:,2)*180/pi, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('\gamma [deg]'); grid on

subplot(3,5,4)
plot(t, R.Mach, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('Mach [-]'); grid on

subplot(3,5,5)
plot(t, R.alpha_deg, 'LineWidth', 1.5); hold on
plot(t, R.alpha_cmd_deg, '--', 'LineWidth', 1.0)
xlabel('Time [s]'); ylabel('\alpha [deg]'); grid on
legend('\alpha actual', '\alpha command', 'Location', 'best')

subplot(3,5,6)
plot(t, R.g_load_earth, 'LineWidth', 1.5)
yline(P.ctrl.g_target, '--', 'g_{target}')
xlabel('Time [s]'); ylabel('g-load [Earth g]'); grid on

subplot(3,5,7)
plot(t, R.q_conv/1e6, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('q_{conv} [MW/m^2]'); grid on

subplot(3,5,8)
plot(t, x(:,5), 'LineWidth', 1.5); hold on
plot(t, x(:,6), 'LineWidth', 1.5)
yline(P.limits.T_inner_max, '--', 'T_{inner,max}')
xlabel('Time [s]'); ylabel('Temperature [K]'); grid on
legend('TPS lump', 'Inner wall', 'Location', 'best')

subplot(3,5,9)
plot(t, x(:,7)*1000, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('TPS thickness [mm]'); grid on

subplot(3,5,10)
plot(t, R.L/1000, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('Lift [kN]'); grid on

subplot(3,5,11)
plot(t, R.rho, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('\rho [kg/m^3]'); grid on

subplot(3,5,12)
plot(t, R.D/1000, 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('Drag [kN]'); grid on

subplot(3,5,13)
plot(t, x(:,10), 'LineWidth', 1.5); hold on
yline(P.vehicle.S_ref_nom, '--', 'A_{nom}')
yline(P.ctrl.A_min, ':', 'A_{min}')
yline(P.ctrl.A_max, ':', 'A_{max}')
xlabel('Time [s]'); ylabel('A_{eff} [m^2]'); grid on

subplot(3,5,14)
plot(t, x(:,9), 'LineWidth', 1.5)
xlabel('Time [s]'); ylabel('CdA_{chute} [m^2]'); grid on

subplot(3,5,15)
plot(t, R.q_dyn, 'LineWidth', 1.5 )
xlabel('Time [s]'); ylabel('P_{dyn} [Pa]'); grid on

sg = sgtitle(sprintf('%s - Traiettoria, Carichi Aerotermici e Termica TPS', P.vehicle.name));
set(sg, 'Color', 'k', 'FontWeight', 'bold')

ax = findall(gcf, 'Type', 'axes');
set(ax, ...
    'Color', 'w', ...
    'XColor', 'k', ...
    'YColor', 'k', ...
    'GridColor', [0.75 0.75 0.75], ...
    'MinorGridColor', [0.85 0.85 0.85], ...
    'FontName', 'Helvetica')

txt = findall(gcf, 'Type', 'text');
set(txt, 'Color', 'k')

% Add a small y-margin so nearly constant signals do not visually collapse
% onto the subplot borders.
for ax_i = ax.'
    yl = ylim(ax_i);
    if all(isfinite(yl))
        yr = yl(2) - yl(1);
        if yr <= 0
            yr = max(abs(yl(1)), 1);
        end
        ylim(ax_i, [yl(1) - 0.08*yr, yl(2) + 0.08*yr]);
    end
end

% -------------------------
% TABELLA RISULTATI
% -------------------------
% Ricerca indice CBM jettison nella history (cambio massa)
% Il jettison avviene a h <= 11740 m & Mach <= 2.07 (dynamics_entry).
% Dopo l'evento P.vehicle.m = m_postCBM; cerchiamo il primo indice con
% Mach <= 2.07 e h <= 11750 per identificare il punto di jettison.
idx_cbm = NaN;
if isfield(P.vehicle, 'm_initial') && isfield(P.vehicle, 'm_postCBM')
    for k = 2:numel(t)
        h_k = max(x(k,3), 0);
        v_k = max(x(k,1), 1);
        atm_k = mars_atmosphere(h_k, P);
        M_k = v_k / atm_k.a;
        if h_k <= 11750 && M_k <= 2.07
            idx_cbm = k;
            break;
        end
    end
end

% Ricerca transizione subsonica
idx_sub = NaN;
if ~isnan(phase_info.t_subchute)
    [~, idx_sub] = min(abs(t - phase_info.t_subchute));
end

fprintf('\n');
fprintf('============================================================================================================\n');
fprintf('  TABELLA RISULTATI SIMULAZIONE — %s\n', P.vehicle.name);
fprintf('  m = %.0f kg | D = %.1f m | TPS = %s (%.0f mm) | Paracadute D = %.1f m\n', ...
    P.vehicle.m_initial, P.vehicle.D, P.tps.material, P.tps.thickness0*1000, P.chute.diameter);
fprintf('============================================================================================================\n');
fprintf('  %-35s | %7s | %9s | %7s | %6s | %6s | %s\n', ...
    'Fase di Volo / Evento', 't [s]', 'h [m]', 'v [m/s]', 'M [-]', 'a [deg]', 'Parametri Chiave');
fprintf('------------------------------------------------------------------------------------------------------------\n');

% 1) Entry Interface
atm0 = mars_atmosphere(P.init.h0, P);
M0 = P.init.v0 / atm0.a;
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.1f | %6.1f | gamma0 = %.1f deg\n', ...
    'Entry Interface (EI)', 0.0, P.init.h0, P.init.v0, M0, x(1,8), x(1,2)*180/pi);

% 2) Picco flusso termico
M_q = R.Mach(idx_q_peak);
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | q_conv,max = %.1f kW/m^2, Ttps = %.0f K\n', ...
    'Picco Flusso Termico', t(idx_q_peak), x(idx_q_peak,3), x(idx_q_peak,1), ...
    M_q, x(idx_q_peak,8), q_peak/1e3, x(idx_q_peak,5));

% 3) Picco decelerazione
M_g = R.Mach(idx_g_peak);
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | a_max = %.2f g\n', ...
    'Picco di Decelerazione', t(idx_g_peak), x(idx_g_peak,3), x(idx_g_peak,1), ...
    M_g, x(idx_g_peak,8), g_peak);

% 4) Picco temperatura esterna TPS
M_tps = R.Mach(idx_tps_peak);
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | T_tps = %.0f K (%.0f C)\n', ...
    'Picco Temp. Esterna TPS', t_tps_peak, h_tps_peak, v_tps_peak, ...
    M_tps, x(idx_tps_peak,8), T_tps_peak, T_tps_peak - 273.15);

% 5) Picco temperatura interna parete
M_in = R.Mach(idx_inner_peak);
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | T_inner = %.0f K (%.1f C)\n', ...
    'Picco Temp. Interna Parete', t_inner_peak, h_inner_peak, v_inner_peak, ...
    M_in, x(idx_inner_peak,8), T_inner_peak, T_inner_peak - 273.15);

% 6) CBM jettison
if ~isnan(idx_cbm)
    M_cbm = R.Mach(idx_cbm);
    fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | m: %.0f -> %.0f kg\n', ...
        'Jettison CBM', t(idx_cbm), x(idx_cbm,3), x(idx_cbm,1), ...
        M_cbm, x(idx_cbm,8), P.vehicle.m_initial, P.vehicle.m_postCBM);
end

% 5) Fine Fase Entry A
if ~isempty(te1a)
    M_1a = xe1a(end,1) / mars_atmosphere(xe1a(end,3), P).a;
    fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | Ingresso finestra paracadute (h <= %.0f m)\n', ...
        'Fine Fase Entry A', te1a(end), xe1a(end,3), xe1a(end,1), ...
        M_1a, xe1a(end,8), P.edl.h_chute_max/1000);
end

% 6) Deploy Paracadute (Fine Fase Entry B)
if entered_chute_window && ~isempty(te1b)
    M_1b = xe1b(end,1) / mars_atmosphere(xe1b(end,3), P).a;
    fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | Deploy Mach %.2f, espulsione DGB\n', ...
        'Manovra SUL & Deploy Chute', te1b(end), xe1b(end,3), xe1b(end,1), ...
        M_1b, xe1b(end,8), M_1b);
end

% 7) Fine Chute Supersonico
if ~isnan(idx_sub)
    M_sub = R.Mach(idx_sub);
    fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | Transizione al regime subsonico\n', ...
        'Fine Chute Supersonico', t(idx_sub), x(idx_sub,3), x(idx_sub,1), ...
        M_sub, x(idx_sub,8));
end

% 8) Fine Simulazione
M_end = R.Mach(end);
fprintf('  %-35s | %7.1f | %9.0f | %7.1f | %6.2f | %6.1f | T_inner = %.1f C, Consumo TPS: %.3f mm\n', ...
    'Quota di Arresto Simulazione', t(end), x(end,3), x(end,1), ...
    M_end, x(end,8), x(end,6)-273.15, thickness_consumed*1000);

fprintf('============================================================================================================\n');

% -------------------------
% PICCHI TERMICI
% -------------------------
fprintf('\n==================================================\n');
fprintf('       PICCHI TERMICI                              \n');
fprintf('==================================================\n');
fprintf('  T_tps esterna max   : %.2f K (%.2f C)\n', T_tps_peak, T_tps_peak - 273.15);
fprintf('    @ t = %.2f s, h = %.0f m, v = %.1f m/s\n', t_tps_peak, h_tps_peak, v_tps_peak);
fprintf('  T_inner parete max  : %.2f K (%.2f C)\n', T_inner_peak, T_inner_peak - 273.15);
fprintf('    @ t = %.2f s, h = %.0f m, v = %.1f m/s\n', t_inner_peak, h_inner_peak, v_inner_peak);
fprintf('  Picco g-load        : %.2f g @ t = %.2f s, h = %.0f m\n', ...
    g_peak, t(idx_g_peak), x(idx_g_peak,3));
fprintf('==================================================\n');

if enable_animation
    animate_entry_attitude(t, x, R);
end
