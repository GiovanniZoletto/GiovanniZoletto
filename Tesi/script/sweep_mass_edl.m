% SWEEP_MASS_EDL: Parametric sweep of entry mass to demonstrate the
% Mars Altitude-Velocity Cliff and the physical limits of parachute landing.

clear; clc; close all;

m_vec = 2000:250:10000; % [kg] range of entry vehicle masses
N = numel(m_vec);

h_deploy_vec = nan(N, 1);
t_deploy_vec = nan(N, 1);
g_peak_vec = nan(N, 1);
q_peak_vec = nan(N, 1);
beta_vec = nan(N, 1);
status_vec = strings(N, 1);

fprintf('=========================================================================\n');
fprintf('  SCANSIONE PARAMETRICA DELLA MASSA: MARS ENTRY & ALTITUDE CLIFF        \n');
fprintf('=========================================================================\n');
fprintf(' %-8s | %-12s | %-12s | %-10s | %-10s | %-15s\n', ...
    'Massa[kg]', 'Beta[kg/m^2]', 'h_deploy[m]', 'g_peak[g]', 'q_peak[kW/m2]', 'Stato');
fprintf('-------------------------------------------------------------------------\n');

for k = 1:N
    P = build_params();
    P.vehicle.m = m_vec(k);
    
    % Ballistic coefficient estimate at hypersonic trim (Cd ~ 1.5)
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
        
        % Check if Mach 2.1 was achieved (Event 1 in Phase B)
        if ~isempty(ie1b) && ie1b(end) == 1
            h_deploy_vec(k) = xe1b(end, 3);
            t_deploy_vec(k) = te1b(end);
            
            if h_deploy_vec(k) >= 8000
                status_vec(k) = "Sicuro (Nominale)";
            elseif h_deploy_vec(k) >= 4000
                status_vec(k) = "Marginale (Critico)";
            else
                status_vec(k) = "Quota Insufficiente";
            end
        else
            status_vec(k) = "Fallito (Crash)";
        end
    else
        t_entry = t1a;
        x_entry = x1a;
        status_vec(k) = "Fallito (No Window)";
    end
    
    % Compute peak loads during entry
    phase_info.idx_entry_end = numel(t_entry);
    phase_info.idx_subchute_start = Inf;
    phase_info.t_supchute = NaN;
    phase_info.t_subchute = NaN;
    
    R = compute_derived_history(t_entry, x_entry, P, phase_info);
    g_peak_vec(k) = max(R.g_load_earth);
    q_peak_vec(k) = max(R.q_conv) / 1e3; % [kW/m^2]
    
    h_str = sprintf('%.0f', h_deploy_vec(k));
    if isnan(h_deploy_vec(k))
        h_str = 'N/A';
    end
    
    fprintf(' %-8.0f | %-12.1f | %-12s | %-10.2f | %-10.1f | %-15s\n', ...
        m_vec(k), beta_vec(k), h_str, g_peak_vec(k), q_peak_vec(k), status_vec(k));
end
fprintf('=========================================================================\n');

% -------------------------
% PLOTTING RESULTS
% -------------------------
figure('Color', 'w', 'Position', [120 100 1100 700]);

subplot(2,2,1)
plot(m_vec, h_deploy_vec / 1000, 'o-', 'LineWidth', 2, 'MarkerFaceColor', [0 0.45 0.74])
yline(8, 'g--', 'Quota Sicura (8 km)', 'LineWidth', 1.5)
yline(4, 'r--', 'Limite Minimo Paracadute (4 km)', 'LineWidth', 1.5)
xline(2804, 'k:', 'MSL (2804 kg)', 'LineWidth', 1.5)
xlabel('Entry Mass [kg]'); ylabel('Parachute Deploy Altitude [km]');
title('Mars Altitude-Velocity Cliff'); grid on;

subplot(2,2,2)
plot(m_vec, beta_vec, 's-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2, 'MarkerFaceColor', [0.85 0.33 0.10])
yline(150, 'k--', '\beta Limit Parachute ~150 kg/m^2')
xlabel('Entry Mass [kg]'); ylabel('Ballistic Coefficient \beta [kg/m^2]');
title('Ballistic Coefficient vs Mass'); grid on;

subplot(2,2,3)
plot(m_vec, g_peak_vec, '^-', 'Color', [0.49 0.18 0.56], 'LineWidth', 2, 'MarkerFaceColor', [0.49 0.18 0.56])
xlabel('Entry Mass [kg]'); ylabel('Peak Load [Earth g]');
title('Peak Deceleration vs Mass'); grid on;

subplot(2,2,4)
plot(m_vec, q_peak_vec, 'd-', 'Color', [0.64 0.08 0.18], 'LineWidth', 2, 'MarkerFaceColor', [0.64 0.08 0.18])
xlabel('Entry Mass [kg]'); ylabel('Peak Heat Flux [kW/m^2]');
title('Peak Stagnation Heat Flux vs Mass'); grid on;

sgtitle('Analisi di Scalabilita della Massa per Rientro Atmosferico su Marte', 'FontWeight', 'bold');
