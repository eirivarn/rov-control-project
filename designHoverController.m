function [Kx, Ki] = designHoverController(Qx, Qu, Qi)
  % DESIGNHOVERCONTROLLER  LQR+integrators for the ROV+actuators model
  %
  % [Kx,Ki] = designHoverController(Qx,Qu,Qi)
  %   Qx  : state–cost matrix (n×n), default = Iₙ
  %   Qu  : input–cost matrix (m×m), default = 0.1·Iₘ
  %   Qi  : integrator–cost scalar, default = 10

  % 1) load the 20-state (12 hydro +8 actuator) model
  [~, A, B, C, ~] = rovWithActuators();

  % 2) sizes
  n  = size(A,1);      % =20
  m  = size(B,2);      % =8
  ni = 6;              % number of integrators (one per reference channel)

  % 3) defaults
  if nargin<1 || isempty(Qx)
    Qx = eye(n);
  end
  if nargin<2 || isempty(Qu)
    Qu = 0.1 * eye(m);
  end
  if nargin<3 || isempty(Qi)
    Qi = 10;
  end

  % 4) build augmented plant with integrators on the first 6 outputs
  C_int = C(1:ni,:);    % integrate the first 6 states (e.g. position/orientation errors)

  Ai = [ A,           zeros(n,ni);
        -C_int,      zeros(ni) ];

  Bi = [ B;
         zeros(ni,m) ];

  % 5) cost matrices
  Q = blkdiag(Qx, Qi*eye(ni));   % (n+ni)×(n+ni)
  R = Qu;                        % m×m

  % 6) solve LQR
  Kfull = lqr(Ai, Bi, Q, R);

  % 7) split into state‐feedback and integrator gains
  Kx = Kfull(:, 1:n);           % m×n
  Ki = Kfull(:, n+1:end);       % m×ni
end
