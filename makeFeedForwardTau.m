% makeFeedForwardTau.m
% Build τ_ff = M_R*nu̇ + D*nu  (added mass + damping only)

% 1) load the test‐trajectory
load('testTrajectory_nu_split.mat', ...
     'ts_u','ts_v','ts_w','ts_p','ts_q','ts_r');
t  = ts_u.Time;  
nu = [ts_u.Data, ts_v.Data, ts_w.Data, ...
      ts_p.Data, ts_q.Data, ts_r.Data];   % N×6

% 2) parameters
params = rovParameters();  % returns .mass, .I, .Ma_lin, .Ma_rot, .D_lin, .D_quad

% 3) compute nu̇ with centered differences
dt = t(2)-t(1);
nu_dot = zeros(size(nu));
nu_dot(1,:)       = (nu(2,:) - nu(1,:)) / dt;
nu_dot(end,:)     = (nu(end,:) - nu(end-1,:)) / dt;
nu_dot(2:end-1,:) = (nu(3:end,:) - nu(1:end-2,:)) / (2*dt);

% 4) loop to form τ_ff
N      = numel(t);
tau_ff = zeros(N,6);
for k = 1:N
  % rigid‐body + added‐mass
  M_rb = [params.mass*eye(3), zeros(3);
          zeros(3),           params.I];
  M_A  = -diag([params.Ma_lin; params.Ma_rot]);
  M_R  = M_rb + M_A;

  % hydrodynamic damping D = D_lin + D_quad * |nu|
  D_lin  = diag(params.D_lin);
  D_quad = diag(params.D_quad .* abs(nu(k,:)'));
  D      = D_lin + D_quad;

  % assemble feed‐forward force/moment
  tau_ff(k,:) = ( M_R*nu_dot(k,:)' + D*nu(k,:)' )';
end

% 5) save as timeseries
ts_tau_ff = timeseries(tau_ff, t, 'Name','tau_ff');
ts_tau_ff.TimeInfo.Units = 'seconds';
save('feedForwardTau.mat','tau_ff');
