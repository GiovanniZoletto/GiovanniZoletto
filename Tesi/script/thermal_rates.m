function [dT_tpsdt, dT_innerdt, dthicknessdt, q_conv, q_rad, q_cond] = thermal_rates(v, rho, T_tps, T_inner, thickness_tps, P, q_conv_override)
% - external aerodynamic heating
% - radiative cooling from hot TPS surface
% - conductive heat transfer through remaining TPS thickness
% - simplified ablative sink active above an effective threshold

if nargin < 7
    q_conv_override = [];
end

thickness_eff = max(thickness_tps, P.tps.thickness_min);

if isempty(q_conv_override)
    [q_conv, q_rad] = aeroheating_mars(v, rho, T_tps, P);
else
    q_conv = q_conv_override;
    q_rad = P.tps.epsilon * 5.670374419e-8 * max(T_tps^4 - P.heat.T_space^4, 0);
end

q_net_surface = max(q_conv - q_rad, 0);
q_cond = P.tps.k * (T_tps - T_inner) / thickness_eff;  % [W/m^2]

dT_trans = 40;
if isfield(P.tps, 'dT_trans')
    dT_trans = P.tps.dT_trans;
end
sigma_abl = 1 / (1 + exp(-(T_tps - P.tps.T_abl_start) / dT_trans));
q_abl = sigma_abl * P.tps.eta_abl * q_net_surface;

% Areal thermal capacities [J/(m^2 K)]
C_tps = P.tps.rho * P.tps.cp * thickness_eff; % 
C_inner = P.wall.rho * P.wall.cp * P.wall.thickness;

dT_tpsdt = (q_net_surface - q_abl - q_cond) / C_tps;   % [K/s]
dT_innerdt = q_cond / C_inner;                         
dthicknessdt = -q_abl / (P.tps.rho * P.tps.H_eff);         % [m/s]
