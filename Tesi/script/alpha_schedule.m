function alpha_deg = alpha_schedule(M, P)
% Prescribed alpha(M) schedule.
%
% Physical interpretation:
% alpha does NOT change automatically because speed changes.
% In this model, alpha(M) is a simplified surrogate for an active guidance
% and attitude-control policy that commands different trim-like attitudes
% in different Mach regimes.

M_clip = min(max(M, P.alpha_sched.M_nodes(1)), P.alpha_sched.M_nodes(end));     % necessary because nodes for alpha scheduling are different from M allowed values
alpha_deg = interp1(P.alpha_sched.M_nodes, P.alpha_sched.alpha_nodes_deg, ...
    M_clip, 'makima'); 
end 
