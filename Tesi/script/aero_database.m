function [Cd, Cl, CA, CN] = aero_database(M, alpha_deg, P)
% Bilinear aerodynamic interpolation on body-axes CA(M, alpha) and CN(M, alpha),
% with exact transformation to wind-axes Drag (Cd) and Lift (Cl).

M_clip = min(max(M, P.aero.M_grid(1)), P.aero.M_grid(end));
alpha_clip = min(max(alpha_deg, P.aero.alpha_grid_deg(1)), P.aero.alpha_grid_deg(end));

if isfield(P.aero, 'CA_table') && isfield(P.aero, 'CN_table')
    CA = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.CA_table, ...
        alpha_clip, M_clip, 'linear');
    CN = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.CN_table, ...
        alpha_clip, M_clip, 'linear');

    alpha_rad = deg2rad(alpha_clip);

    % Exact Body Frame -> Wind Frame Transformation:
    % Cd =  CA * cos(alpha) + CN * sin(alpha)
    % Cl = -CA * sin(alpha) + CN * cos(alpha)
    Cd = CA * cos(alpha_rad) + CN * sin(alpha_rad);
    Cl = -CA * sin(alpha_rad) + CN * cos(alpha_rad);
else
    % Fallback for legacy Cd/Cl tables
    Cd = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.Cd_table, ...
        alpha_clip, M_clip, 'linear');
    Cl = interp2(P.aero.alpha_grid_deg, P.aero.M_grid, P.aero.Cl_table, ...
        alpha_clip, M_clip, 'linear');
    CA = Cd;
    CN = Cl;
end
end
