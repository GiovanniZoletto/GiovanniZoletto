% SIMULATE_SRP_DESCENT: Standalone script to simulate Supersonic Retro-Propagation (SRP)
% for heavy/human landers (Red Dragon 9.7t / NASA HIAD 15t) down to touchdown (h=0).

clear; clc; close all;

% -------------------------------------------------
% 1) Vehicle selection and SRP configuration
% -------------------------------------------------
% Choose vehicle: 'red_dragon' | 'hiad_human'
vehicle_choice = 'hiad_human'; % <-- change to 'red_dragon' if needed

if strcmp(vehicle_choice, 'red_dragon')
    P = build_params_red_dragon();
    h_ignition = 6000;      % [m] lowered ignition altitude (orig 8000 m)
    M_ignition = 2.8;       % [Mach]
    T_engine   = 540e3;      % [N] total thrust (8 SuperDraco)
    Isp        = 260;       % [s] MMH/NTO
    m_prop     = 2500 + 50; % [kg] original + reserve
else
    P = build_params_hiad_human();
    h_ignition = 3000;      % [m] lowered ignition altitude (orig 4000 m)
    M_ignition = 1.2;       % [Mach]
    T_engine   = 200e3;      % [N] total thrust (4 LOX/CH4)
    Isp        = 350;       % [s] LOX/CH4
    m_prop     = 4500 + 50; % [kg] original + reserve
end

% -------------------------------------------------
% 2) Initial state (same as entry simulation)
% -------------------------------------------------
atm0   = mars_atmosphere(P.init.h0, P);
alpha0 = alpha_schedule(P.init.v0 / atm0.a, P);

x0 = [P.init.v0; P.init.gamma0; P.init.h0; P.init.s0; ...
      P.init.T_tps0; P.init.T_inner0; P.init.thickness_tps0; alpha0; 0; P.init.A_eff0];

% -------------------------------------------------
% 3) PHASE 1 – Hyper‑sonic unpowered entry until SRP trigger
% -------------------------------------------------
opts_entry = odeset('RelTol',1e-6,'AbsTol',1e-7,...
    'Events',@(t,x) event_srp_trigger(t,x,P,h_ignition,M_ignition));

[t1,x1,te1,xe1,ie1] = ode45(@(t,x) dynamics_entry(t,x,P),[0,P.sim.tmax_entry],x0,opts_entry);

fprintf('\n=========================================================================\n');
fprintf('  SIMULAZIONE RIENTRO PROPULSIVO SUPERSUONICO (SRP) - %s \n',P.vehicle.name);
fprintf('========================================================================= \n');
fprintf('  Fine Fase Ipersonica (Accensione SRP) : t = %6.2f s | Quota = %6.1f m | Mach = %4.2f | v = %5.1f m/s\n',...
    te1(end), xe1(end,3), xe1(end,1)/mars_atmosphere(xe1(end,3),P).a, xe1(end,1));

% -------------------------------------------------
% 4) PHASE 2 – Powered retro‑propulsion down to touchdown
% -------------------------------------------------
% State vector extended with remaining propellant mass
x0_srp = [xe1(end,1); xe1(end,2); xe1(end,3); xe1(end,4); xe1(end,5); xe1(end,6); m_prop];

opts_srp = odeset('RelTol',1e-6,'AbsTol',1e-7,'Events',@(t,x) event_touchdown(t,x));

[t2,x2,te2,xe2,ie2] = ode45(@(t,x) dynamics_srp(t,x,P,T_engine,Isp),[te1(end), te1(end)+300],x0_srp,opts_srp);

% -------------------------------------------------
% 5) OUTPUT – Touchdown and temperature extrema
% -------------------------------------------------
fprintf('  Touchdown / Fine SRP                  : t = %6.2f s | Quota = %6.1f m | v = %5.1f m/s | Prop. rimasto = %5.1f kg\n',...
    te2(end), xe2(end,3), xe2(end,1), xe2(end,7));

% Maximum external (TPS) temperature during the whole trajectory (entry + SRP)
T_tps_all = [x1(:,5); x2(:,5)];
[max_T_tps, idx_max_tps] = max(T_tps_all);
t_all = [t1; t2 + te1(end)];
time_max_tps = t_all(idx_max_tps);

% Maximum inner wall temperature
T_inner_all = [x1(:,6); x2(:,6)];
[max_T_inner, idx_max_inner] = max(T_inner_all);
time_max_inner = t_all(idx_max_inner);

fprintf('  Temperatura esterna (TPS) massima     : %6.2f K (%.2f °C)  @ t = %5.2f s\n', max_T_tps, max_T_tps-273.15, time_max_tps);
fprintf('  Temperatura parete interna massima    : %6.2f K (%.2f °C)  @ t = %5.2f s\n', max_T_inner, max_T_inner-273.15, time_max_inner);
fprintf('=========================================================================\n');

% -------------------------------------------------
% LOCAL FUNCTIONS
% -------------------------------------------------
function [val, term, dir] = event_srp_trigger(~, x, P, h_ign, M_ign)
    v = x(1); h = x(3);
    atm = mars_atmosphere(h,P);
    M = v/atm.a;
    % Trigger when altitude ≤ h_ign AND Mach ≤ M_ign (or safety floor at 500 m)
    val = [h - h_ign; M - M_ign; h - 500];
    term = [1;1;1];
    dir  = [-1;-1;-1];
end

function [val, term, dir] = event_touchdown(~, x)
    h = x(3); v = x(1);
    % Touchdown: ground (h=0) or practically zero speed
    val = [h; v - 0.5];
    term = [1;1];
    dir  = [-1;-1];
end

function dxdt = dynamics_srp(~, x, P, T_thrust, Isp_s)
    v = max(x(1),0.1); gamma = x(2); h = max(x(3),0);
    T_tps = x(5); T_inner = x(6); m_p = max(x(7),0);

    % Current total mass = dry vehicle mass + remaining propellant
    m_curr = P.vehicle.m + m_p;
    g0 = 9.80665;
    atm = mars_atmosphere(h,P);
    g   = mars_gravity(h,P);
    q_dyn = 0.5 * atm.rho * v^2;
    Cd = 1.20;                     % conservative drag coefficient for SRP phase
    D  = q_dyn * Cd * P.vehicle.S_ref;

    % Throttle logic
    if h > 500
        T = T_thrust;                         % full thrust
    else
        T = min(T_thrust, m_curr * (g + 2.0));% soft‑landing thrust
    end
    if m_p <= 0, T = 0; end               % propellant exhausted

    mdot = -T / (Isp_s * g0);             % propellant consumption (kg/s)

    dvdt = -(D + T) / m_curr - g * sin(gamma);
    dgammadt = -(g / v - v / (P.mars.R + h)) * cos(gamma);
    dhdt = v * sin(gamma);
    dsdt = v * cos(gamma) * P.mars.R / (P.mars.R + h);

    % Thermal rates (reuse existing function)
    [dT_tpsdt, dT_innerdt, ~] = thermal_rates(v, atm.rho, T_tps, T_inner, P.tps.thickness0, P, 0);

    dxdt = [dvdt; dgammadt; dhdt; dsdt; dT_tpsdt; dT_innerdt; mdot];
end