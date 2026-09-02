function atm = mars_atmosphere(h, P)
% Mars atmosphere model based on NASA Glenn curve fits.
% Source:
% https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/mars-atmosphere-equation-metric/
%
% Important note:
% the simple Glenn temperature fit becomes nonphysical at very high
% altitude because it can predict T <= 0 K. For entry simulations starting
% around 125 km, we keep the pressure law but clamp temperature to a
% reasonable floor so the sound speed remains real and the ODE stays well
% posed.

h = max(h, 0);

if h < 7000
    T_c = -31 - 0.000998 * h;      % [degC]
    p_kPa = 0.699 * exp(-0.00009 * h);
else
    T_c = -23.4 - 0.00222 * h;     % [degC]
    p_kPa = 0.699 * exp(-0.00009 * h);
end

T = max(T_c + 273.15, 130);        % [K] floor added for high-altitude robustness
p = p_kPa * 1000;                  % [Pa]
rho = p / (P.mars.Rgas * T);       % [kg/m^3]
a = sqrt(P.mars.gamma_gas * P.mars.Rgas * T); % [m/s]

atm.T = T;
atm.p = p;
atm.rho = rho;
atm.a = a;
end
