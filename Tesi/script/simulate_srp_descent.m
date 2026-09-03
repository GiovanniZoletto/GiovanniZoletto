% SIMULATE_SRP_DESCENT: Standalone script to simulate Supersonic Retro-Propulsion
% (SRP) for heavy/human landers (Red Dragon 9.7t / NASA HIAD 15t) down to touchdown (h=0).

clear; clc; close all;

% Select vehicle: 'red_dragon' | 'hiad_human'
vehicle_choice = 'hiad_human';

if strcmp(vehicle_choice, 'red_dragon')
    P = build_params_red_dragon();
    h_ignition = 8000;  % [m] ignition altitude for SuperDraco SRP
    M_ignition = 2.8;   % [Mach] supersonic retro-propulsion threshold
    T_engine = 540e3;   % [N] 8 SuperDraco total thrust
    Isp = 260;          % [s] MMH/NTO
    m_prop = 2500;      % [kg] available propellant
else
    P = build_params_hiad_human();
    h_ignition = 4000;  % [m] ignition altitude after HIAD deceleration
    M_ignition = 1.2;   % [Mach]
    T_engine = 200e3;   % [N] 4 LOX/CH4 engines
    Isp = 350;          % [s] LOX/CH4
    m_prop = 4500;      % [kg]
end

atm0 = mars_atmosphere(P.init.h0, P);
alpha0 = alpha_schedule(P.init.v0 / atm0.a, P);

x0 = [P.init.v0; P.init.gamma0; P.init.h0; P.init.s0; P.init.T_tps0; ...
      P.init.T_inner0; P.init.thickness_tps0; alpha0; 0; P.init.A_eff0];

% -------------------------
% PHASE 1: HYPERSONIC UNPOWERED ENTRY
% -------------------------
opts_entry = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, ...
    'Events', @(t,x) event_srp_trigger(t, x, P, h_ignition, M_ignition));

[t1, x1, te1, xe1, ie1] = ode45(@(t,x) dynamics_entry(t, x, P), ...
    [0, P.sim.tmax_entry], x0, opts_entry);

fprintf('=========================================================================\n');
fprintf('  SIMULAZIONE RIENTRO PROPULSIVO SUPERSUONICO (SRP) - %s\n', P.vehicle.name);
fprintf('=========================================================================\n');
fprintf('  Fine Fase Ipersonica (Accensione SRP) : t = %6.2f s | Quota = %6.1f m | Mach = %4.2f | v = %5.1f m/s\n', ...
    te1(end), xe1(end,3), xe1(end,1)/mars_atmosphere(xe1(end,3),P).a, xe1(end,1));

% -------------------------
% PHASE 2: POWERED RETRO-PROPULSION (SRP) DOWN TO TOUCHDOWN
% -------------------------
% State vector extended with propellant mass: x_srp = [v; gamma; h; s; T_tps; T_inner; m_prop]
x0_srp = [xe1(end,1); xe1(end,2); xe1(end,3); xe1(end,4); xe1(end,5); xe1(end,6); m_prop];

opts_srp = odeset('RelTol', 1e-6, 'AbsTol', 1e-7, ...
    'Events', @(t,x) event_touchdown(t, x));

[t2, x2, te2, xe2, ie2] = ode45(@(t,x) dynamics_srp(t, x, P, T_engine, Isp), ...
    [te1(end), te1(end) + 200], x0_srp, opts_srp);

fprintf('  Touchdown / Fine SRP                  : t = %6.2f s | Quota = %6.1f m | v = %5.1f m/s | Prop. rimasto = %5.1f kg\n', ...
    te2(end), xe2(end,3), xe2(end,1), xe2(end,7));
fprintf('  Temperatura Parete Interna Finale     : %6.2f K (%.2f °C)\n', ...
    xe2(end,6), xe2(end,6) - 273.15);
fprintf('=========================================================================\n');

% -------------------------
% LOCAL FUNCTIONS
% -------------------------
function [val, term, dir] = event_srp_trigger(~, x, P, h_ign, M_ign)
    v = x(1); h = x(3);
    atm = mars_atmosphere(h, P);
    M = v / atm.a;
    val = [h - h_ign; M - M_ign; h - 500];
    term = [1; 1; 1];
    dir = [-1; -1; -1];
end

function [val, term, dir] = event_touchdown(~, x)
    v = x(1); h = x(3);
    val = [h; v - 2.0]; % Touchdown on ground or zero velocity
    term = [1; 1];
    dir = [-1; -1];
end

function dxdt = dynamics_srp(~, x, P, T_thrust, Isp_s)
    v = max(x(1), 0.1); gamma = x(2); h = max(x(3), 0);
    T_tps = x(5); T_inner = x(6); m_p = max(x(7), 0);
    
    m_curr = (P.vehicle.m - m_p) + m_p;
    g0 = 9.80665;
    
    atm = mars_atmosphere(h, P);
    g = mars_gravity(h, P);
    q_dyn = 0.5 * atm.rho * v^2;
    Cd = 1.20;
    D = q_dyn * Cd * P.vehicle.S_ref;
    
    % Throttle logic: decelerate smoothly to touchdown
    if h > 500
        T = T_thrust;
    else
        T = min(T_thrust, m_curr * (g + 2.0)); % Soft touchdown thrust
    end
    
    if m_p <= 0
        T = 0; % Propellant exhausted
    end
    
    mdot = -T / (Isp_s * g0);
    
    dvdt = -(D + T) / m_curr - g * sin(gamma);
    dgammadt = -(g / v - v / (P.mars.R + h)) * cos(gamma);
    dhdt = v * sin(gamma);
    dsdt = v * cos(gamma) * P.mars.R / (P.mars.R + h);
    
    [dT_tpsdt, dT_innerdt, ~] = thermal_rates(v, atm.rho, T_tps, T_inner, P.tps.thickness0, P, 0);
    
    dxdt = [dvdt; dgammadt; dhdt; dsdt; dT_tpsdt; dT_innerdt; mdot];
end
