function P = build_params()
% Centralized parameter file for the MSL-like Mars entry model.
%
% Each parameter block is labeled as:
%   source   -> taken from public references
%   estimate -> engineering estimate chosen for a preliminary model
%
% Main public references used:
% 1) MSL aerodynamic challenges:
%    https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20090025343.pdf
% 2) MSL EDL performance / comparison table:
%    https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070016022.pdf
% 3) Mars atmosphere model:
%    https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/mars-atmosphere-equation-metric/
% 4) Mars facts:
%    https://science.nasa.gov/mars/facts/
% 5) NASA low-density ablators / PICA overview:
%    https://www.nasa.gov/general/thermal-protection-materials-branch-low-density-ablators/
% 6) Mars 2020 parachute deployment illustration:
%    https://www.nasa.gov/image-article/perseverance-deploys-its-parachute/

% -------------------------
% MARS PLANETARY CONSTANTS
% -------------------------
P.mars.R = 3389.5e3;        % [m] source: NASA Mars facts
P.mars.mu = 4.2828e13;      % [m^3/s^2] source: NASA Mars facts / standard Mars GM
P.mars.Rgas = 192.1;        % [J/(kg K)] source: NASA Glenn Mars atmosphere model
P.mars.gamma_gas = 1.29;    % [-] estimate for CO2-dominated atmosphere in this range

% -------------------------
% VEHICLE GEOMETRY / MASS
% -------------------------
P.vehicle.name = 'MSL-like 70 deg sphere-cone';
P.vehicle.D = 4.5;                              % [m] source: MSL papers
P.vehicle.S_ref = pi * (P.vehicle.D^2) / 4;     % [m^2] derived from diameter
P.vehicle.S_ref_nom = P.vehicle.S_ref;          % [m^2] nominal effective aero area
P.vehicle.R_n = 1.125;                          % [m] estimate: blunt forebody nose radius ~ D/4
P.vehicle.m = 2804;                             % [kg] source: MSL comparison tables
P.vehicle.LD_target_hyp = 0.24;                 % [-] source: MSL papers

% -------------------------
% ENTRY INITIAL CONDITIONS
% -------------------------
P.init.v0 = 5600;                  % [m/s] source: MSL comparison table / rounded
P.init.gamma0 = -11*pi/180;        % [rad] estimate consistent with prior Mars entry studies
P.init.h0 = 125000;                % [m] estimate / standard top-of-model atmosphere
P.init.s0 = 0;                     % [m]

% Thermal initial conditions
P.init.T_tps0 = 220;               % [K] estimate: cold-soaked initial external TPS state
P.init.T_inner0 = 293;             % [K] estimate: cabin-side wall near room temperature
P.init.CdA_chute0 = 0;             % [m^2] no chute drag before deployment
P.init.A_eff0 = P.vehicle.S_ref_nom; % [m^2] initial effective aero area

% -------------------------
% AERODYNAMIC DATABASE (Body Axes CA, CN & Wind Transformation)
% -------------------------
% Aerodynamic tables for 70 deg sphere-cone entry capsule tabulated in
% body-axes Axial Force CA(M,alpha) and Normal Force CN(M,alpha).
% Wind-axis forces are calculated exactly as:
%   Cd =  CA * cos(alpha) + CN * sin(alpha)
%   Cl = -CA * sin(alpha) + CN * cos(alpha)
%
% source basis:
% - MSL aerodynamic challenges paper (NASA/TM-2009-215343)
% - Phoenix capsule aerodynamics paper (NASA/TM-2008-215348)
P.aero.M_grid = [0.8 1.2 2 5 10 15 20 25];
P.aero.alpha_grid_deg = [-20 -16 -12 -8 -4 0 4 8 12 16 20];

% Axial force coefficient CA(M, alpha)
P.aero.CA_table = [ ...
    0.82 0.81 0.80 0.80 0.80 0.80 0.80 0.80 0.80 0.81 0.82; ...
    0.91 0.90 0.89 0.88 0.88 0.88 0.88 0.88 0.89 0.90 0.91; ...
    1.15 1.13 1.11 1.09 1.08 1.08 1.08 1.09 1.11 1.13 1.15; ...
    1.48 1.45 1.42 1.39 1.36 1.35 1.36 1.39 1.42 1.45 1.48; ...
    1.60 1.57 1.53 1.49 1.46 1.45 1.46 1.49 1.53 1.57 1.60; ...
    1.62 1.59 1.55 1.51 1.48 1.47 1.48 1.51 1.55 1.59 1.62; ...
    1.63 1.60 1.56 1.52 1.49 1.48 1.49 1.52 1.56 1.60 1.63; ...
    1.64 1.61 1.57 1.53 1.50 1.49 1.50 1.53 1.57 1.61 1.64];

% Normal force coefficient CN(M, alpha)
P.aero.CN_table = [ ...
   -0.16 -0.13 -0.10 -0.07 -0.03  0.00  0.03  0.07  0.10  0.13  0.16; ...
   -0.18 -0.15 -0.11 -0.07 -0.04  0.00  0.04  0.07  0.11  0.15  0.18; ...
   -0.20 -0.16 -0.12 -0.08 -0.04  0.00  0.04  0.08  0.12  0.16  0.20; ...
   -0.22 -0.18 -0.14 -0.09 -0.05  0.00  0.05  0.09  0.14  0.18  0.22; ...
   -0.24 -0.19 -0.14 -0.10 -0.05  0.00  0.05  0.10  0.14  0.19  0.24; ...
   -0.24 -0.19 -0.14 -0.10 -0.05  0.00  0.05  0.10  0.14  0.19  0.24; ...
   -0.23 -0.18 -0.14 -0.09 -0.05  0.00  0.05  0.09  0.14  0.18  0.23; ...
   -0.23 -0.18 -0.14 -0.09 -0.05  0.00  0.05  0.09  0.14  0.18  0.23];

P.aero.trim_alpha_deg = -16.0;      % [deg] nominal hypersonic trim angle of attack
P.aero.bank_angle_deg = 65.0;       % [deg] nominal bank angle for vertical lift modulation

% -------------------------
% PRESCRIBED ALPHA(M) SCHEDULE & SUL MANEUVER
% -------------------------
% Hypersonic entry holds trim alpha = -16 deg.
% Right before parachute deploy (Mach 2.1-2.5), the Straighten-Up and
% Level-Forward (SUL) maneuver transitions alpha to 0 deg.
P.alpha_sched.M_nodes = [0.8 1.5 2.1 2.5 5 10 15 20 25];
P.alpha_sched.alpha_nodes_deg = [0 0 0 -16 -16 -16 -16 -16 -16];
P.alpha_sched.tau_alpha = 4.0;      % [s] attitude adjustment time constant

% -------------------------
% THERMAL PROTECTION SYSTEM (PICA-like)
% -------------------------
P.tps.material = 'PICA-like';
P.tps.rho = 270;                   % [kg/m^3] source: NASA PICA density ~0.27 g/cm^3
P.tps.k = 0.06;                    % [W/(m K)] estimate: representative low conductivity
P.tps.cp = 1500;                   % [J/(kg K)] estimate: representative effective cp
P.tps.epsilon = 0.90;              % [-] estimate: high-emissivity carbonaceous surface
P.tps.thickness0 = 0.060;              % [m] estimate: preliminary 60 mm forebody TPS
P.tps.thickness_min = 0.005;           % [m] estimate: avoid singular thermal resistance
P.tps.T_abl_start = 800;           % [K] estimate: effective onset for strong ablation sink
P.tps.dT_trans = 40;               % [K] transition width for continuous sigmoid ablation
P.tps.H_eff = 2.0e7;               % [J/kg] estimate: effective ablation enthalpy
P.tps.eta_abl = 0.65;              % [-] estimate: fraction of net heat consumed by ablation
P.init.thickness_tps0 = P.tps.thickness0;  % [m] initial remaining TPS thickness

% -------------------------
% INNER WALL / STRUCTURE
% -------------------------
% Because detailed MSL wall-stack data are not openly available, the inner
% wall is modeled as an equivalent areal thermal mass.
P.wall.description = 'Equivalent inner wall, effective material';
P.wall.rho = 1600;                 % [kg/m^3] estimate
P.wall.cp = 900;                   % [J/(kg K)] estimate
P.wall.thickness = 0.012;          % [m] estimate

% -------------------------
% THERMAL / HEATING MODEL
% -------------------------
% Simplified Mars entry stagnation-point-like heating model:
% q_conv = C_heat * sqrt(rho / R_n) * V^3
%
% source basis:
% - Sutton-Graves style blunt-body heating relation
% - updated Mars stagnation-point correlation literature
% estimate:
% - C_heat selected for preliminary Mars entry magnitudes
P.heat.C_heat = 1.5e-4;            % [SI mixed coefficient] estimate
P.heat.T_space = 210;              % [K] estimate for radiative sink reference
P.heat.A_heat = 0.70 * P.vehicle.S_ref; % [m^2] estimate: heated forebody area fraction

% -------------------------
% LIMITS
% -------------------------
P.limits.T_inner_max = 30 + 273.15;        % [K] 303.15 K (30 degC) max cabin-side interior temperature
P.limits.T_inner_abort = 50 + 273.15;      % [K] 313.15 K (40 degC) emergency stop threshold

% -------------------------
% AREA CONTROL
% -------------------------
% The effective reference area is used as a simple control actuator.
% This is a modeling abstraction for a variable-drag / variable-aero-
% authority concept, not a literal stock MSL mechanism.
P.ctrl.A_min = 0.70 * P.vehicle.S_ref_nom;   % [m^2] estimate
P.ctrl.A_max = 1.30 * P.vehicle.S_ref_nom;   % [m^2] estimate
P.ctrl.tau_A = 8;                            % [s] estimate: area-actuator time constant
P.ctrl.g_target = 4.0;                       % [Earth g] control target
P.ctrl.T_inner_target = 27 + 273.15;         % [K] 300.15 K (27 degC) control target below abort
P.ctrl.T_tps_target = 850;                   % [K] estimate: anticipatory TPS temperature target
P.ctrl.q_target = 2.2e5;                     % [W/m^2] estimate: heating guidance target
P.ctrl.Kg = 1.4;                             % [m^2 / g] estimate
P.ctrl.KT_inner = 0.06;                      % [m^2 / K] estimate
P.ctrl.KT_tps = 0.01;                        % [m^2 / K] estimate
P.ctrl.Kq = 1.2e-5;                          % [m^2 / (W/m^2)] estimate

% -------------------------
% EDL EVENTS
% -------------------------
P.edl.Mach_chute_deploy = 2.1;     % [-] source: MSL/Persverance deploy regime
P.edl.Mach_subchute = 0.8;         % [-] estimate: second chute stage after subsonic transition
P.edl.h_chute_max = 11000;         % [m] source: NASA Mars 2020 illustration
P.edl.h_stop = 2000;               % [m] user-requested stop before active landing phase

P.chute.diameter = 21.5;           % [m] source: NASA Mars 2020 parachute size
P.chute.area = pi * (P.chute.diameter^2) / 4; % [m^2]
P.chute.Cd_sup = 0.22;             % [-] estimate: reefed / partial supersonic stage
P.chute.Cd_sub = 0.55;             % [-] estimate: fuller subsonic stage
P.chute.CdA_sup = P.chute.Cd_sup * P.chute.area;
P.chute.CdA_sub = P.chute.Cd_sub * P.chute.area;
P.chute.tau_deploy_sup = 0.5;        % [s] estimate: supersonic canopy growth time
P.chute.tau_deploy_sub = 0.3;        % [s] estimate: subsonic opening time

% -------------------------
% NUMERICAL SETTINGS
% -------------------------
P.sim.tmax_entry = 700;            % [s] estimate: enough to reach deploy or thermal limit
P.sim.tmax_total = 1200;           % [s] estimate

end
