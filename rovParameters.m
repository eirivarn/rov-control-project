function params = rovParameters()
% ROVPARAMETERS  Return a struct of physical parameters for the ROV.

  % Mass and inertia
  params.mass       = 50;                  % kg
  params.I          = diag([2, 3, 4]);     % kg·m^2

  % Added-mass (positive values)
  params.Ma_lin     = [20;20;20];          % [Xu_dot; Yv_dot; Zw_dot]
  params.Ma_rot     = [1;1;1];             % [Kp_dot; Mq_dot; Nr_dot]

  % Damping coefficients (linear and quadratic)
  params.D_lin      = [30;30;30;2;2;2];     % [Xu; Yv; Zw; Kp; Mq; Nr]
  params.D_quad     = [40;40;40;5;5;5];     % [Xu|u|; Yv|v|; Zw|w|; ... ]

  % Buoyancy / hydrostatic
  params.rho        = 1025;                % kg/m^3
  params.g          = 9.81;                % m/s^2
  params.volume     = 0.05;                % m^3 (displaced water)

  % CG/CB offsets (if you implement restoring moments)
  params.rG         = [0;0;0];             % m
  params.rB         = [0;0;0.1];           % m
end
