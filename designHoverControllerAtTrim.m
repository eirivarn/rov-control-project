function [Kx, Ki] = designHoverControllerAtTrim(A, B, Qx, Qu, Qi)
% DESIGNHOVERCONTROLLERATTRIM  LQR+integrator gains for 6-state model
%
% Inputs:
%   A  – n×n (here n=6)
%   B  – n×m (here m=6)
%   Qx – n×n
%   Qu – m×m
%   Qi – scalar
% Outputs:
%   Kx – m×n
%   Ki – m×6

  % sizes
  n  = size(A,1);    % should be 6
  m  = size(B,2);    % should be 6
  ni = 6;            % number of integrators

  % integrator on the first ni outputs
  C_int = eye(ni,n);

  % augmented plant (n+ni states, m inputs)
  Ai = [ A,           zeros(n,ni);
        -C_int,      zeros(ni) ];
  Bi = [ B;
         zeros(ni,m) ];

  % cost matrices (now correct size: (n+ni)×(n+ni))
  Q = blkdiag(Qx, Qi*eye(ni));  
  R = Qu;

  % solve LQR
  Kfull = lqr(Ai, Bi, Q, R);

  % partition
  Kx = Kfull(:, 1:n);
  Ki = Kfull(:, n+1:end);
end
