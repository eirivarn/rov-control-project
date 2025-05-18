function eta_dot = computeKinematicsMap(~, eta, nu)
  [R,T] = computeRotationMatrices(eta(4), eta(5), eta(6));
  J = [R, zeros(3); zeros(3), T];
  eta_dot = J * nu;
end
