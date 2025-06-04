function nu_dot = computeDynamicsSimple(nu, tau, eta)
% Simplified 6-DOF + small restoring moments for roll/pitch (no heave)
%
% Inputs:
%   nu  [6×1], tau [6×1], eta [6×1]
%
  %#codegen
  p = rovParameters();

  % Rigid‐body inertia
  M_rb = [ p.mass*eye(3), zeros(3);
           zeros(3),       p.I ];

  % Flip vertical thrust
  tau(3) = -tau(3);

  % Drag
  D    = diag(p.D_lin) + diag(p.D_quad .* abs(nu));
  drag = D * nu;

  % Small roll/pitch restoring (no heave)
  phi   = eta(4);
  theta = eta(5);
  K_phi   = p.mass * 0.1;   % small stiffness gains
  K_theta = p.mass * 0.1;
  g = zeros(6,1);
  g(4) = -K_phi   * phi;  
  g(5) = -K_theta * theta;

  % Acceleration
  nu_dot = M_rb \ (tau - drag - g);
end
