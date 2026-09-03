function dxdt = dynamics_entry(~, x, P)
% Entry-phase 3DOF translational dynamics with prescribed alpha(M)
% and thermal / area-control model.
%
% State vector:
% x = [v; gamma; h; s; T_tps; T_inner; thickness_tps; alpha; CdA_chute; A_eff]

v = max(x(1), 1);
gamma = x(2);
h = max(x(3), 0);
T_tps = x(5);
T_inner = x(6);
thickness_tps = max(x(7), P.tps.thickness_min);
alpha_deg = x(8);
CdA_chute = x(9);
A_eff = x(10);

atm = mars_atmosphere(h, P);
g = mars_gravity(h, P);
M = v / atm.a;

alpha_cmd_deg = alpha_schedule(M, P); % commanded alpha from the guidance schedule
[Cd, Cl] = aero_database(M, alpha_deg, P);

q_dyn = 0.5 * atm.rho * v^2; % Dynamic pressure
D = q_dyn * Cd * A_eff;  
L_3D = q_dyn * Cl * A_eff;

% Vertical lift projection with Bank Angle sigma:
bank_angle_deg = 65.0;
if isfield(P.aero, 'bank_angle_deg')
    bank_angle_deg = P.aero.bank_angle_deg;
end
sigma_rad = deg2rad(bank_angle_deg);
L_vert = L_3D * cos(sigma_rad);

% Point-mass entry dynamics on a spherical planet.
dvdt = -D / P.vehicle.m - g * sin(gamma);
dgammadt = L_vert / (P.vehicle.m * v) - (g / v - v / (P.mars.R + h)) * cos(gamma);
dhdt = v * sin(gamma);
dsdt = v * cos(gamma) * P.mars.R / (P.mars.R + h);

[dT_tpsdt, dT_innerdt, dthicknessdt, q_conv] = thermal_rates(v, atm.rho, T_tps, T_inner, thickness_tps, P);
g_load = sqrt(D^2 + L_3D^2) / (P.vehicle.m * 9.81);
A_cmd = area_controller(A_eff, g_load, T_tps, T_inner, q_conv, P); % commanded effective area
dalpha_dt = (alpha_cmd_deg - alpha_deg) / P.alpha_sched.tau_alpha;
dCdA_dt = 0;
dA_dt = (A_cmd - A_eff) / P.ctrl.tau_A;

dxdt = [dvdt; dgammadt; dhdt; dsdt; dT_tpsdt; dT_innerdt; dthicknessdt; dalpha_dt; dCdA_dt; dA_dt];
end
