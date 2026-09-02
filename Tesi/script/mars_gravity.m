function g = mars_gravity(h, P)
% Variable gravity model:
% g(h) = mu / (R + h)^2
% Source basis: NASA Mars planetary constants.

r = P.mars.R + max(h, 0);
g = P.mars.mu / r^2;
end
