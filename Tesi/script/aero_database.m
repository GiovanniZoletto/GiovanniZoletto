function [Cd, Cl] = aero_database(M, alpha_deg, P)
% Bilinear aerodynamic interpolation on surrogate MSL-like tables.

M_clip = min(max(M, P.aero.M_grid(1)), P.aero.M_grid(end));
alpha_clip = min(max(alpha_deg, P.aero.alpha_grid_deg(1)), ...
    P.aero.alpha_grid_deg(end)); % clip between allowed values

Cd = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.Cd_table, ...
    alpha_clip, M_clip, 'linear');
Cl = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.Cl_table, ...
    alpha_clip, M_clip, 'linear');  %interpolate to get corresponding Cd and Cl
end
