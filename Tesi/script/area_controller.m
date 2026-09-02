function A_cmd = area_controller(A_eff, g_load, T_tps, T_inner, q_conv, P)
% Simple effective-area controller.
%
% Sign logic:
% - if g-load is too high, reduce area to reduce aerodynamic force
% + if inner/TPS temperature or heating are too high, increase area to
%   brake earlier and reduce future heating exposure

u_g = -P.ctrl.Kg * max(0, g_load - P.ctrl.g_target);
u_inner = P.ctrl.KT_inner * max(0, T_inner - P.ctrl.T_inner_target);
u_tps = P.ctrl.KT_tps * max(0, T_tps - P.ctrl.T_tps_target);
u_q = P.ctrl.Kq * max(0, q_conv - P.ctrl.q_target);

A_cmd = P.vehicle.S_ref_nom + u_g + u_inner + u_tps + u_q;
A_cmd = min(max(A_cmd, P.ctrl.A_min), P.ctrl.A_max);
end
