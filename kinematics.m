function deta = kinematics(~, eta, nu)
% KINEMATICS  Compute time derivative of pose eta
% eta = [x; y; z; φ; θ; ψ], nu = [u; v; w; p; q; r]

  % unpack
  phi   = eta(4);
  theta = eta(5);
  psi   = eta(6);

  % rotation blocks
  [R, T] = rotationMatrices(phi, theta, psi);

  % assemble Jacobian J(eta)
  J = [ R,       zeros(3);
        zeros(3),    T     ];

  % kinematic map
  deta = J * nu;
end
