% SWEEP_MASS_EDL: Parametric sweep of entry mass and geometry
% Demonstrates the Mars Altitude-Velocity Cliff and parachute landing limits.

clear; clc; close all;

% Mode: 'constant_diameter' (MSL 4.5m) or 'scaled_diameter' (D up to 10m)
mode = 'scaled_diameter'; 

m_vec = 2000:500:15000; % [kg] range of entry masses up to human landing scale
N = numel(m_vec);

h_deploy_vec = nan(N, 1);
t_deploy_vec = nan(N, 1);
g_peak_vec = nan(N, 1);
q_peak_vec = nan(N, 1);
beta_vec = nan(N, 1);
D_vec = nan(N, 1);
D_chute_vec = nan(N, 1);
status_vec = strings(N, 1);

fprintf('========================================================================================\n');
fprintf('  SCANSIONE PARAMETRICA: MARS ENTRY, ALTITUDE CLIFF E SCALABILITA (Modo: %s)\n', mode);
fprintf('========================================================================================\n');
fprintf(' %-8s | %-6s | %-8s | %-12s | %-12s | %-10s | %-15s\n', ...
    'Massa[kg]', 'D[m]', 'D_ch[m]', 'Beta[kg/m2]', 'h_deploy[m]', 'g_peak[g]', 'Stato');
fprintf('----------------------------------------------------------------------------------------\n');

for k = 1:N
    P = build_params();
    m_k = m_vec(k);
    P.vehicle.m = m_k;
    
    if strcmp(mode, 'scaled_diameter')
        % Scale aeroshell diameter with cubic root of mass (constant density), capped at 10m
        D_k = min(4.5 * (m_k / 2804)^(1/3), 10.0);
        P.vehicle.D = D_k;
        P.vehicle.S_ref = pi * (D_k^2) / 4;
        P.vehicle.S_ref_nom = P.vehicle.S_ref;
        P.vehicle.R_n = D_k / 4;
        P.init.A_eff0 = P.vehicle.S_ref;
        
        % Scale parachute diameter with square root of mass, capped at 35m
        D_chute_k = min(21.5 * sqrt(m_k / 2804), 35.0);
        P.chute.diameter = D_chute_k;
        P.chute.area = pi * (D_chute_k^2) / 4;
        P.chute.CdA_sup = P.chute.Cd_sup * P.chute.area;
        P.chute.CdA_sub = P.chute.Cd_sub * P.chute.area;
    else
        D_k = P.vehicle.D;
        D_chute_k = P.chute.diameter;
    end
    
    D_vec(k) = D_k;
    D_chute_vec(k) = D_chute_k;
    
    % Ballistic coefficient (hypersonic drag Cd ~ 1.5)
    Cd_hyp = 1.50;
    beta_vec(k) = P.vehicle.m / (Cd_hyp * P.vehicle.S_ref);
    
    atm0 = mars_atmosphere(P.init.h0, P);
    alpha0 = alpha_schedule(P.init.v0 / atm0.a, P);
    
    x0 = [ ...
        P.init.v0;
        P.init.gamma0;
        P.init.h0;
        P.init.s0;
        P.init.T_tps0;
        P.init.T_inner0;
        P.init.thickness_tps0;
        alpha0;
        P.init.CdA_chute0;
        P.init.A_eff0];
        
    opts_a = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, ...
        'Events', @(t,x) event_entry_phase_a(t, x, P));
        
    [t1a, x1a, te1a, xe1a, ie1a] = ode45(@(t,x) dynamics_entry(t, x, P), ...
        [0, P.sim.tmax_entry], x0, opts_a);
        
    entered_chute_win = ~isempty(ie1a) && ie1a(end) == 1;
    
    if entered_chute_win
        opts_b = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, ...
            'Events', @(t,x) event_entry_phase_b(t, x, P));
            
        [t1b, x1b, te1b, xe1b, ie1b] = ode45(@(t,x) dynamics_entry(t, x, P), ...
            [te1a(end), P.sim.tmax_entry], xe1a(end,:).', opts_b);
            
        t_entry = [t1a; t1b(2:end)];
        x_entry = [x1a; x1b(2:end,:)];
        
        if ~isempty(ie1b) && ie1b(end) == 1
            h_deploy_vec(k) = xe1b(end, 3);
            t_deploy_vec(k) = te1b(end);
            
            if h_deploy_vec(k) >= 8000
                status_vec(k) = "Sicuro (Nominale)";
            elseif h_deploy_vec(k) >= 4000
                status_vec(k) = "Critico (Marginale)";
            else
                status_vec(k) = "Quota Insufficiente";
            end
        else
            status_vec(k) = "Crash (Mach 2 al suolo)";
        end
    else
        t_entry = t1a;
        x_entry = x1a;
        status_vec(k) = "Fallito";
    end
    
    phase_info.idx_entry_end = numel(t_entry);
    phase_info.idx_subchute_start = Inf;
    phase_info.t_supchute = NaN;
    phase_info.t_subchute = NaN;
    
    R = compute_derived_history(t_entry, x_entry, P, phase_info);
    g_peak_vec(k) = max(R.g_load_earth);
    q_peak_vec(k) = max(R.q_conv) / 1e3;
    
    h_str = sprintf('%.0f', h_deploy_vec(k));
    if isnan(h_deploy_vec(k))
        h_str = 'N/A';
    end
    
    % Compute Δv due to CBM jettison if it occurs in this simulation
    % We run a short entry simulation to capture velocity before and after mass change
    P_tmp = P; % copy parameters
    % Run entry phase A & B to get velocity at mass change point
    opts_a = odeset('RelTol',1e-6,'AbsTol',1e-7,'Events',@(t,x) event_entry_phase_a(t,x,P_tmp));
    [~,~,~,~,ie_a] = ode45(@(t,x) dynamics_entry(t,x,P_tmp), [0,P_tmp.sim.tmax_entry], x0, opts_a);
    % Assuming mass change occurs in phase B (post-entry window)
    v_before = NaN; v_after = NaN;
    if ~isempty(ie_a) && ie_a(end)==1
        % Run phase B up to mass change point
        opts_b = odeset('RelTol',1e-6,'AbsTol',1e-7,'Events',@(t,x) event_entry_phase_b(t,x,P_tmp));
        [t_b, x_b, ~, ~, ie_b] = ode45(@(t,x) dynamics_entry(t,x,P_tmp), [0,P_tmp.sim.tmax_entry], x0, opts_b);
        % Find index where mass changed (h <= 11740 & M <= 2.07)
        for idx=1:size(x_b,1)
            h = x_b(idx,3); v = x_b(idx,1);
            atm = mars_atmosphere(h,P_tmp);
            M = v/atm.a;
            if h <= 11740 && M <= 2.07
                v_before = v; % velocity just before change
                % after change velocity will be next step (if exists)
                if idx < size(x_b,1)
                    v_after = x_b(idx+1,1);
                else
                    v_after = v;
                end
                break;
            end
        end
    end
    delta_v = v_before - v_after;
    if isnan(delta_v); delta_v = 0; end

    % Store for table output
    delta_v_vec(k) = delta_v; % [m/s]

    % Update table printing to include post-CBM mass (constant) and Δv
    fprintf(' %-8.0f | %-6.2f | %-8.1f | %-12.1f | %-12s | %-10.2f | %-8.1f | %-15s\n', ...
        m_vec(k), D_vec(k), D_chute_vec(k), beta_vec(k), h_str, g_peak_vec(k), delta_v_vec(k), status_vec(k));
end
fprintf('========================================================================================\n');

% -------------------------
% PLOTTING
% -------------------------
figure('Color', 'w', 'Position', [100 80 1150 750]);

subplot(2,2,1)
plot(m_vec, h_deploy_vec / 1000, 'o-', 'LineWidth', 2, 'Color', [0 0.45 0.74], 'MarkerFaceColor', [0 0.45 0.74])
yline(8, 'g--', 'Quota Sicura (8 km)', 'LineWidth', 1.5)
yline(4, 'r--', 'Limite Minimo (4 km)', 'LineWidth', 1.5)
xline(2804, 'k:', 'MSL (2804 kg)', 'LineWidth', 1.5)
xlabel('Entry Mass [kg]'); ylabel('Parachute Deploy Altitude [km]');
title('Mars Altitude-Velocity Cliff'); grid on;

subplot(2,2,2)
plot(m_vec, beta_vec, 's-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2, 'MarkerFaceColor', [0.85 0.33 0.10])
yline(150, 'k--', '\beta Limit Parachute ~150 kg/m^2')
xlabel('Entry Mass [kg]'); ylabel('Ballistic Coefficient \beta [kg/m^2]');
title('Ballistic Coefficient vs Mass'); grid on;

subplot(2,2,3)
plot(m_vec, D_vec, '^-', 'Color', [0.47 0.67 0.19], 'LineWidth', 2, 'MarkerFaceColor', [0.47 0.67 0.19])
xlabel('Entry Mass [kg]'); ylabel('Aeroshell Diameter [m]');
title('Aeroshell Diameter vs Mass'); grid on;

subplot(2,2,4)
plot(m_vec, g_peak_vec, 'd-', 'Color', [0.49 0.18 0.56], 'LineWidth', 2, 'MarkerFaceColor', [0.49 0.18 0.56])
xlabel('Entry Mass [kg]'); ylabel('Peak Load [Earth g]');
title('Peak Deceleration vs Mass'); grid on;

sgtitle('Analisi di Scalabilita EDL su Marte con Riscalamento Dimensionale', 'FontWeight', 'bold');
