function [value, isterminal, direction] = event_entry_phase_b(~, x, P)
% Entry phase B stops on:
% 1) crossing below the parachute deployment Mach threshold
% 2) stop altitude
% 3) emergency inner-wall thermal limit

v = max(x(1), 1);
h = x(3);
T_inner = x(6);

atm = mars_atmosphere(h, P);
M = v / atm.a;

value = [M - P.edl.Mach_chute_deploy; h - P.edl.h_stop; ...
    T_inner - P.limits.T_inner_abort];

isterminal = [1; 1; 1];
direction = [-1; -1; +1];
end
