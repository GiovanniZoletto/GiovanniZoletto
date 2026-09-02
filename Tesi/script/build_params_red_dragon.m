function P = build_params_red_dragon()
% Centralized parameter file for the SpaceX Red Dragon Mars entry concept.
%
% Reference:
% NASA-TM-2017-219504 ("Red Dragon Mars Entry and Descent Simulation")
% SpaceX / NASA Ames Joint Study on Supersonic Retro-Propulsion

% Base physical and atmospheric parameters
P = build_params();

% -------------------------
% VEHICLE GEOMETRY / MASS (Red Dragon)
% -------------------------
P.vehicle.name = 'SpaceX Red Dragon Concept (9.7t)';
P.vehicle.D = 3.70;                              % [m] Dragon capsule max diameter
P.vehicle.S_ref = pi * (P.vehicle.D^2) / 4;     % [m^2] ~10.75 m^2
P.vehicle.S_ref_nom = P.vehicle.S_ref;
P.vehicle.R_n = 1.50;                          % [m] Dragon spherical nose radius
P.vehicle.m = 9700;                             % [kg] Entry interface mass (Dragon + 2500kg propellant)
P.vehicle.LD_target_hyp = 0.26;                 % [-] Hypersonic L/D for Dragon at trim alpha ~ -18 deg

% Initial conditions
P.init.A_eff0 = P.vehicle.S_ref_nom;

% Area control bounds scaled to Dragon
P.ctrl.A_min = 0.70 * P.vehicle.S_ref_nom;
P.ctrl.A_max = 1.30 * P.vehicle.S_ref_nom;
P.heat.A_heat = 0.70 * P.vehicle.S_ref;

% -------------------------
% TPS (PICA-X)
% -------------------------
P.tps.material = 'PICA-X (SpaceX 3rd Gen)';
P.tps.rho = 270;                   % [kg/m^3]
P.tps.k = 0.065;                   % [W/(m K)]
P.tps.cp = 1550;                   % [J/(kg K)]
P.tps.thickness0 = 0.045;          % [m] 45 mm nominal heatshield thickness
P.init.thickness_tps0 = P.tps.thickness0;

% -------------------------
% SUPERSONIC RETRO-PROPULSION (SRP) PARAMETERS
% -------------------------
P.srp.num_engines = 8;             % 8 SuperDraco engines
P.srp.thrust_total = 540e3;        % [N] 540 kN total thrust (8 x 67.5 kN)
P.srp.Isp = 260;                   % [s] specific impulse for MMH/NTO in Mars atmosphere
P.srp.m_propellant = 2500;         % [kg] onboard propellant budget for descent

end
