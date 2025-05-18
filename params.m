% params.m

% Simulation timing
dt     = 0.01;    % [s] fixed step for Euler, unused by ode45
Tfinal = 10;      % [s] total sim time

% Body‐rate command (constant for now)
u = 1; v = 0; w = 0;    % surge, sway, heave [m/s]
p = 0.1; q = 0.05; r = 0;  % roll, pitch, yaw rates [rad/s]
nu = [u; v; w; p; q; r];
