%% 1) load the test‐trajectory (with η in it)
load('testTrajectory_nu_full.mat', ...
     'ts_u','ts_v','ts_w','ts_p','ts_q','ts_r','ts_eta');

t    = ts_u.Time;                            % N×1
nu   = [ts_u.Data, ts_v.Data, ts_w.Data, ...
        ts_p.Data, ts_q.Data, ts_r.Data];    % N×6
eta  = ts_eta.Data;                          % N×6
N    = numel(t);

%% 2) parameters
p = rovParameters();   % flat struct as before

%% 3) compute ν̇ with centered differences
dt     = t(2) - t(1);
nu_dot = zeros(size(nu));
nu_dot(1,:)       = (nu(2,:)   - nu(1,:)  ) / dt;
nu_dot(end,:)     = (nu(end,:) - nu(end-1,:)) / dt;
nu_dot(2:end-1,:) = (nu(3:end,:)- nu(1:end-2,:)) / (2*dt);

%% 4) loop to form τ_ff_simple
tau_ff = zeros(N,6);

% precompute rigid-body inertia
M_rb = [ p.mass*eye(3), zeros(3);
         zeros(3),      p.I ];
M_A  = diag([p.Ma_lin; p.Ma_rot]);
M_R  = M_rb + M_A;

invM = inv(M_rb);

for k = 1:N
    % 4.1 drag D(nu)
    D_lin  = diag(p.D_lin);
    D_quad = diag(p.D_quad .* abs(nu(k,:)'));
    D      = D_lin + D_quad;
    
    % 4.2 restoring g(eta)
    phi   = eta(k,4);
    theta = eta(k,5);
    eps_ang = 1e-3;   % ~0.057° buffer
    theta = max(min(theta,  pi/2 - eps_ang), -pi/2 + eps_ang);
    W     = p.mass * p.g;
    B     = p.rho * p.g * p.volume;
    dW    = W - B;
    g = zeros(6,1);
    g(1) =  dW * sin(theta);
    g(2) = -dW * cos(theta) * sin(phi);
    g(3) = -dW * cos(theta) * cos(phi);
    
    % 4.3 assemble τ_ff_simple
    tau_ff(k,:) = ( M_R  * nu_dot(k,:)' ...
                  + D     * nu(k,:)' ...
                  + g )';
end

%% 5) save as timeseries
ts_tau_ff = timeseries(tau_ff, t, 'Name','tau_ff_simple');
ts_tau_ff.TimeInfo.Units = 'seconds';
save('feedForwardTau_simple.mat','tau_ff');
