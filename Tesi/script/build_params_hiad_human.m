function P = build_params_hiad_human()
% Centralized parameter file for the NASA Human Mars Landing with HIAD concept.
%
% References:
% 1) NASA Special Publication SP-2009-566-ADD2 (Human Exploration of Mars DRA 5.0)
% 2) NASA LOFTID Flight Demonstration Overview (2022)

% Base physical and atmospheric parameters
P = build_params();

% -------------------------
% VEHICLE GEOMETRY / MASS (HIAD Human Lander)
% -------------------------
P.vehicle.name = 'NASA Human Mars Lander (15t HIAD 10m)';
P.vehicle.D = 10.0;                              % [m] Inflatable HIAD diameter
P.vehicle.S_ref = pi * (P.vehicle.D^2) / 4;     % [m^2] ~78.54 m^2
P.vehicle.S_ref_nom = P.vehicle.S_ref;
P.vehicle.R_n = 2.50;                          % [m] Nose radius ~ D/4
P.vehicle.m = 15000;                            % [kg] Entry interface mass (crewed habitat + descent stage)
P.vehicle.LD_target_hyp = 0.24;                 % [-] Hypersonic L/D for 70 deg HIAD sphere-cone

% Initial conditions
P.init.A_eff0 = P.vehicle.S_ref_nom;

% Area control bounds and gains scaled to HIAD
scale_A = P.vehicle.S_ref_nom / (pi * 4.5^2 / 4);
P.ctrl.A_min = 0.70 * P.vehicle.S_ref_nom;
P.ctrl.A_max = 1.30 * P.vehicle.S_ref_nom;
P.ctrl.Kg = 1.4 * scale_A;
P.ctrl.KT_inner = 0.06 * scale_A;
P.ctrl.KT_tps = 0.01 * scale_A;
P.ctrl.Kq = 1.2e-5 * scale_A;
P.ctrl.g_target = 5.0;                          % [Earth g] crew-safe target for HIAD
P.heat.A_heat = 0.70 * P.vehicle.S_ref;

% -------------------------
% FLEXIBLE TPS (F-TPS)
% -------------------------
P.tps.material = 'F-TPS (Nextel / Pyrogel / Kapton)';
P.tps.rho = 200;                   % [kg/m^3] flexible aerogel/fabric density
P.tps.k = 0.040;                   % [W/(m K)] ultra-low conductivity aerogel
P.tps.cp = 1400;                   % [J/(kg K)]
P.tps.thickness0 = 0.025;          % [m] 25 mm flexible thermal blanket
P.init.thickness_tps0 = P.tps.thickness0;

% -------------------------
% POWERED DESCENT RETRO-PROPULSION
% -------------------------
P.srp.num_engines = 4;             % 4 LOX/Methane throttleable engines
P.srp.thrust_total = 200e3;        % [N] 200 kN total thrust
P.srp.Isp = 350;                   % [s] specific impulse for LOX/CH4
P.srp.m_propellant = 4500;         % [kg] propellant budget

end
