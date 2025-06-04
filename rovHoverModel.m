function [sys, A, B, C, D] = rovHoverModel()
% ROVHOVERMODEL  Build the linearized hover state‐space model
%
%   [sys,A,B,C,D] = rovHoverModel() loads the vehicle parameters (including
%   trim and hydrostatic stiffness), then assembles the 12×12 A‐matrix with
%   small‐angle restoring terms and the 12×6 B‐matrix for control‐input
%   perturbations, and returns the corresponding ss(sys).

  %#codegen
  % load parameters (mass, added‐mass, damping, G, etc.)
  p = rovParameters();
  
  % total inertia and linear damping from params
  M  = p.M_total;   % 6×6 = [M_rb + M_a]
  D0 = p.D0;        % 6×6 linear damping
  G  = p.G;         % 6×6 hydrostatic stiffness

  % build A, B, C, D
  A = [ zeros(6),      eye(6);
       -M\G,         -M\D0 ];
  B = [ zeros(6,6);
        M\eye(6)    ];
  C = eye(12);
  D = zeros(12,6);

  % assemble linear system
  sys = ss(A,B,C,D);
end
