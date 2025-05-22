function [Kx, Ki] = designHoverControllerAtTrim(A, B, Qx, Qu, Qi)
% designHoverControllerAtTrim  LQR+integrator gains for a given linearized plant
%
% [Kx,Ki] = designHoverControllerAtTrim(A,B,Qx,Qu,Qi)
%   A,B    : linearized state‐space at the trim point
%   Qx     : state-cost (n×n)
%   Qu     : input-cost (m×m)
%   Qi     : integrator-cost scalar
%
%   Returns: Kx (m×n), Ki (m×ni) where ni=6

  % sizes
  n  = size(A,1);
  m  = size(B,2);
  ni = 6;  % integrators on the first six outputs

  % build C_int to integrate position/orientation errors (states 1–6)
  C_int = eye(ni,n);

  % augmented plant
  Ai = [ A,           zeros(n,ni);
        -C_int,      zeros(ni)   ];
  Bi = [ B;
         zeros(ni,m) ];

  % cost matrices
  Q = blkdiag(Qx, Qi*eye(ni));  
  R = Qu;

  % solve LQR
  Kfull = lqr(Ai, Bi, Q, R);

  % partition gains
  Kx = Kfull(:, 1:n);
  Ki = Kfull(:, n+1:end);
end
