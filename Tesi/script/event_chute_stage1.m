function [value, isterminal, direction] = event_chute_stage1(~, x, P)
% Stop supersonic chute stage on:
% 1) subsonic threshold
% 2) stop altitude

v = max(x(1), 1);
h = x(3);

atm = mars_atmosphere(h, P);
M = v / atm.a;

value = [M - P.edl.Mach_subchute; h - P.edl.h_stop];

isterminal = [1; 1];
direction = [-1; -1];
end
