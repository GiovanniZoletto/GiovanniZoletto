function [value, isterminal, direction] = event_entry_phase_a(~, x, P)
% Entry phase A stops on:
% 1) crossing below the maximum allowed parachute deployment altitude
% 2) stop altitude 
% 3) emergency inner-wall thermal limit


h = x(3);
T_inner = x(6);

value = [h - P.edl.h_chute_max; h - P.edl.h_stop;...
    T_inner - P.limits.T_inner_abort];

isterminal = [1; 1; 1];    % stop simulation in case the 3 conditions are met
direction = [-1; -1; +1];  % [Go below h_chute_max;Go below h_stop; surpass T_inner abort] 
end
