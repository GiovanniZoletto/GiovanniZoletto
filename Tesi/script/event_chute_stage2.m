function [value, isterminal, direction] = event_chute_stage2(~, x, P)
% Stop subsonic chute stage on:
% 1) stop altitude

h = x(3);

value = h - P.edl.h_stop;

isterminal = 1;
direction = -1;
end
