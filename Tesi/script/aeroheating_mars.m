function [q_conv, q_rad] = aeroheating_mars(v, rho, T_tps, P)
% Simplified Mars entry heating model.
%
% q_conv uses a stagnation-point-like relation with explicit nose radius:
% q_conv = C_heat * sqrt(rho / R_n) * V^3
%
% The model is intentionally simple and preliminary. The coefficient is an
% estimate chosen to keep entry heating in a realistic Mars-mission range.

v = max(v, 1);
rho = max(rho, 0);

q_conv = P.heat.C_heat * sqrt(rho / P.vehicle.R_n) * v^3;  % C_heat to be checked on sources
q_rad = P.tps.epsilon * 5.670374419e-8 * max(T_tps^4 - P.heat.T_space^4, 0);
end
