function R = compute_derived_history(t, x, P, phase_info)
% Recompute derived variables for plotting in a way consistent with the
% dynamics functions.

n = numel(t);

R.Mach = zeros(n,1);
R.alpha_deg = zeros(n,1);
R.alpha_cmd_deg = zeros(n,1);
R.Cd = zeros(n,1);
R.Cl = zeros(n,1);
R.rho = zeros(n,1);
R.L = zeros(n,1);
R.L_vert = zeros(n,1);
R.D = zeros(n,1);
R.CdA_chute = zeros(n,1);
R.A_eff = zeros(n,1);
R.g_load_earth = zeros(n,1);
R.q_conv = zeros(n,1);
R.q_rad = zeros(n,1);
R.q_dyn = zeros(n,1);
R.phase = strings(n,1);

for i = 1:n
    v = max(x(i,1), 1);
    h = max(x(i,3), 0);
    T_tps = x(i,5);
    alpha_deg = x(i,8);
    CdA_chute = max(x(i,9), 0);
    A_eff = x(i,10);

    atm = mars_atmosphere(h, P);
    M = v / atm.a;
    R.Mach(i) = M;
    R.rho(i) = atm.rho;
    R.alpha_cmd_deg(i) = alpha_schedule(M, P);

    if i <= phase_info.idx_entry_end
        [Cd, Cl] = aero_database(M, alpha_deg, P);
        q_dyn = 0.5 * atm.rho * v^2;
        D = q_dyn * Cd * A_eff;
        L = q_dyn * Cl * A_eff;
        [q_conv, q_rad] = aeroheating_mars(v, atm.rho, T_tps, P);
        R.phase(i) = "entry";
    else
        if i < phase_info.idx_subchute_start
            R.phase(i) = "chute_sup";
        else
            R.phase(i) = "chute_sub";
        end

        [Cd_body, Cl_body] = aero_database(M, alpha_deg, P);
        Cd = Cd_body + CdA_chute / max(A_eff, 1e-6);
        Cl = Cl_body;
        q_dyn = 0.5 * atm.rho * v^2;
        D = q_dyn * (Cd_body * A_eff + CdA_chute);
        L = q_dyn * Cl_body * A_eff;
        q_conv = 0;
        q_rad = P.tps.epsilon * 5.670374419e-8 * max(T_tps^4 - P.heat.T_space^4, 0);
    end

    R.alpha_deg(i) = alpha_deg;
    R.Cd(i) = Cd;
    R.Cl(i) = Cl;
    R.L(i) = L;
    bank_angle_deg = 65.0;
    if isfield(P.aero, 'bank_angle_deg')
        bank_angle_deg = P.aero.bank_angle_deg;
    end
    R.L_vert(i) = L * cos(deg2rad(bank_angle_deg));
    R.D(i) = D;
    R.CdA_chute(i) = CdA_chute;
    R.A_eff(i) = A_eff;
    R.g_load_earth(i) = sqrt(D^2 + L^2) / (P.vehicle.m * 9.81);
    R.q_conv(i) = q_conv;
    R.q_rad(i) = q_rad;
    R.q_dyn(i) = q_dyn;
end
end
