function dxdt = dynamics_chute_stage(~, x, P, stage)
% Parachute-phase dynamics with two simplified drag stages.
%
% stage = 1  -> supersonic / reefed chute
% stage = 2  -> subsonic / fuller chute

v = max(x(1), 1);
gamma = x(2);
h = max(x(3), 0);
T_tps = x(5);
T_inner = x(6);
thickness_tps = max(x(7), P.tps.thickness_min);
alpha_deg = x(8);
CdA_chute = max(x(9), 0);
A_eff = x(10);

atm = mars_atmosphere(h, P);
g = mars_gravity(h, P);
q_dyn = 0.5 * atm.rho * v^2;

M = v / atm.a;
[Cd_body, Cl_body] = aero_database(M, alpha_deg, P);
D_body = q_dyn * Cd_body * A_eff;
L_body = q_dyn * Cl_body * A_eff;

if stage == 1
    CdA_target = P.chute.CdA_sup;
    tau_deploy = P.chute.tau_deploy_sup;
else
    CdA_target = P.chute.CdA_sub;
    tau_deploy = P.chute.tau_deploy_sub;
end

D_chute = q_dyn * CdA_chute;
D = D_body + D_chute;

dvdt = -D / P.vehicle.m - g * sin(gamma);
dgammadt = L_body / (P.vehicle.m * v) - (g / v - v / (P.mars.R + h)) * cos(gamma);
dhdt = v * sin(gamma);
dsdt = v * cos(gamma) * P.mars.R / (P.mars.R + h);

% After heatshield separation external forebody heating is suppressed and
% keep only thermal soak / cooldown in this simplified model.
[dT_tpsdt, dT_innerdt, dthicknessdt] = thermal_rates(v, atm.rho, T_tps, T_inner, thickness_tps, P, 0);
dalpha_dt = (0 - alpha_deg) / P.alpha_sched.tau_alpha;
dCdA_dt = (CdA_target - CdA_chute) / tau_deploy;
dA_dt = 0;

dxdt = [dvdt; dgammadt; dhdt; dsdt; dT_tpsdt; dT_innerdt; dthicknessdt; dalpha_dt; dCdA_dt; dA_dt];
end
