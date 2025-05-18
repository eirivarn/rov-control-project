function [R, T] = rotationMatrices(phi, theta, psi)
% ROTATIONMATRICES  Compute R(φ,θ,ψ) and T(φ,θ)
%
% Inputs:
%   phi, theta, psi — Euler angles [rad]
%
% Outputs:
%   R — 3×3 rotation from body→inertial
%   T — 3×3 map from body rates to Euler‐rates

  % full R = Rz(ψ)*Ry(θ)*Rx(φ)
  Rz = [ cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
             0   ,      0   , 1];
  Ry = [ cos(theta), 0, sin(theta);
              0   , 1,      0   ;
        -sin(theta), 0, cos(theta)];
  Rx = [ 1,     0    ,      0   ;
         0, cos(phi), -sin(phi);
         0, sin(phi),  cos(phi)];
  R = Rz * Ry * Rx;

  % T map
  T = [ 1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0,       cos(phi)      ,      -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta) ];
end
