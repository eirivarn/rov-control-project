function [Kx, Ki] = designHoverController(Qx, Qu, Qi)
  if nargin < 1, Qx = eye(12); end
  if nargin < 2, Qu = 0.1 * eye(6); end
  if nargin < 3, Qi = 10; end

  [sys, A, B, C, ~] = rovHoverModel();

  n  = size(A,1);
  ni = 6;
  C_int = C(1:6,:); 
  
  Ai = [ A,        zeros(n,ni);
      -C_int,    zeros(ni) ];
  Bi = [ B;
       zeros(ni, size(B,2)) ];

  Q = blkdiag(Qx, Qi*eye(ni));

  R = Qu;

  Kfull = lqr(Ai, Bi, Q, R);
  Kx = Kfull(:, 1:n);
  Ki = Kfull(:, n+1:end);
end
