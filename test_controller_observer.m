% test_controller_observer.m
clear; clc; close all

%% 1) Build the 20‐state “hydro + actuator” model
[sys_aug, A_aug, B_aug, C_aug, ~, Tmix] = rovWithActuators();
n  = size(A_aug,1);   % 20
m  = size(B_aug,2);   % 8
ni = 6;               % # of integrators (x,y,z,phi,theta,psi)

fprintf('=== Model Dimensions ===\n');
fprintf('A_aug: %d×%d, B_aug: %d×%d, Tmix: %d×%d\n', ...
    size(A_aug), size(B_aug), size(Tmix));

%% 2) Mixing Matrix Rank
rank_Tmix = rank(Tmix);
sv_Tmix   = svd(Tmix);
fprintf('\n=== Mixing Matrix Tmix ===\n');
fprintf('rank(Tmix) = %d (expected 6)\n', rank_Tmix);
fprintf('singular values of Tmix = [%s]\n', sprintf('%6.3f ', sv_Tmix));

%% 3) Reachability & Observability of (A_aug, B_aug, C_aug)
Wc_full  = ctrb(A_aug, B_aug);
rWc_full = rank(Wc_full);
svWc_full = svd(Wc_full);

Wo_full  = obsv(A_aug, C_aug);
rWo_full = rank(Wo_full);
svWo_full = svd(Wo_full);

fprintf('\n=== Gramian Checks (A_aug, B_aug, C_aug) ===\n');
fprintf('Controllability: rank = %d/%d,  min(sv) = %8.4e\n', rWc_full, n, min(svWc_full));
fprintf('Observability:   rank = %d/%d,  min(sv) = %8.4e\n', rWo_full, n, min(svWo_full));
fprintf('Controllability: %d/%d, Observability: %d/%d\n', rWc_full, n, rWo_full, n);

%% 4) Design LQR + integrators
Qx = eye(n);
Qu = 0.1 * eye(m);
Qi = 10;
[Kx, Ki] = designHoverController(Qx, Qu, Qi);

% Build closed‐loop Acl (26×26)
C_int = C_aug(1:ni,:);  % selects [x,y,z,phi,theta,psi]
Acl = [ A_aug - B_aug*Kx,   -B_aug*Ki;
        -C_int,             zeros(ni, ni) ];

fprintf('\n=== Controller Gains ===\n');
fprintf('Kx size: %d×%d,   Ki size: %d×%d\n', size(Kx), size(Ki));

%% 5) Closed‐Loop Eigenvalues & Damping
lambda_cl = eig(Acl);
fprintf('\n=== Closed‐Loop Eigenvalues ===\n');
for i = 1:length(lambda_cl)
    fprintf('  %2d: Re = %+8.4e,  Im = %+8.4e\n', i, real(lambda_cl(i)), imag(lambda_cl(i)));
end
maxRe_cl = max(real(lambda_cl));
if maxRe_cl < 0
    fprintf('Max Re(eig(Acl)) = %+8.4e → stable\n', maxRe_cl);
else
    fprintf('Max Re(eig(Acl)) = %+8.4e → UNSTABLE or marginal\n', maxRe_cl);
end

Acl_ss = ss(Acl, [], eye(size(Acl)), []);
[wn_cl, zeta_cl, ~] = damp(Acl_ss);
fprintf('\nNatural frequencies (rad/s) & damping ratios:\n');
for i = 1:length(wn_cl)
    fprintf('  Mode %2d: wn = %6.3f,  zeta = %5.3f\n', i, wn_cl(i), zeta_cl(i));
end

%% 6) Linear Step‐Response Performance (per DOF)
Bcl_ref = [ zeros(n, ni); eye(ni) ];  % maps 6 references into integrators
Ccl = [ eye(ni), zeros(ni, n) ];      % only [x,y,z,phi,theta,psi]
Dcl = zeros(ni, ni);
sys_cl = ss(Acl, Bcl_ref, Ccl, Dcl);

fprintf('\n=== Linear Step‐Response Metrics ===\n');
ref_mag = 0.1;
Tend = 10; dt = 0.01; t = (0:dt:Tend)';
tol_pct = 0.02;  % 2%

for i = 1:ni
    Uref = zeros(ni, numel(t));
    Uref(i, :) = ref_mag;
    Y = lsim(sys_cl, Uref', t);
    y_i = Y(:, i);
    y_ss = y_i(end);
    err_ss = abs(ref_mag - y_ss);
    y_peak = max(y_i);
    overshoot = (y_peak / ref_mag - 1) * 100;
    tol = tol_pct * ref_mag;
    t_settle = NaN;
    for j = 1:length(t)
        if max(abs(y_i(j:end) - ref_mag)) <= tol
            t_settle = t(j);
            break
        end
    end
    fprintf('DOF %2d: ss error = %8.4e,  overshoot = %5.2f%%,  settling time = %5.2f s\n', ...
            i, err_ss, overshoot, t_settle);
end

%% 7) Design Observer (Kalman/Luenberger) with aggressive Qn/Rn
usePressure     = true;  % z
useIMU          = true;  % phi, theta
useMagnetometer = true;  % psi
useDVL          = true;  % u, v
useSBL          = true;  % x, y

measIdx = [];
Rn_vals = [];
if useSBL
    measIdx(end+1:end+2) = [1,2];      Rn_vals(end+1:end+2) = [1e-1, 1e-1];
end
if usePressure
    measIdx(end+1) = 3;                Rn_vals(end+1) = 1e-5;
end
if useIMU
    measIdx(end+1:end+2) = [4,5];      Rn_vals(end+1:end+2) = [1e-4, 1e-4];
end
if useMagnetometer
    measIdx(end+1) = 6;                Rn_vals(end+1) = 1e-3;
end
if useDVL
    measIdx(end+1:end+2) = [7,8];      Rn_vals(end+1:end+2) = [1e-2, 1e-2];
end

p = numel(measIdx);
C_sens = zeros(p, n);
for k = 1:p
    C_sens(k, measIdx(k)) = 1;
end

% Aggressive process noise and small measurement noise
Qn = 1e0 * eye(n);
Rn = 1e-4 * eye(p);

L_sens = lqe(A_aug, eye(n), C_sens, Qn, Rn);
L_hydro    = L_sens(1:12, :);
L_actuator = L_sens(13:20, :);

A_obs = A_aug - L_sens * C_sens;        % 20×20
B_obs = [ B_aug,   L_sens ];            % 20×(8 + p)
C_obs = eye(n);
D_obs = zeros(n, size(B_obs,2));

fprintf('\n=== Observer Design (Aggressive Qn/Rn) ===\n');
fprintf('C_sens size: %d×%d,   Rn size: %d×%d\n', size(C_sens), size(Rn));
fprintf('L_sens size: %d×%d\n', size(L_sens));

Wo_obs = obsv(A_aug, C_sens);
rWo_obs = rank(Wo_obs);
svWo_obs = svd(Wo_obs);
fprintf('\nObservability (with sensors): rank = %d/%d, min(sv) = %8.4e\n', rWo_obs, n, min(svWo_obs));

eig_obs = eig(A_obs);
slowest_obs = max(real(eig_obs));
fprintf('Slowest observer pole (real part) = %+8.4e\n', slowest_obs);

%% 8) Observer Convergence (linear, zero input) with aggressive gains
Tend_obs = 1; dt_obs = 0.002; t_obs = (0:dt_obs:Tend_obs)';
nt_obs = numel(t_obs);

X_true = zeros(n, nt_obs);
X_est  = zeros(n, nt_obs);
X_true(:,1) = zeros(n,1);
X_est(:,1)  = 5 * ones(n,1);

for k = 1:nt_obs-1
    X_true(:,k+1) = X_true(:,k) + dt_obs * (A_aug * X_true(:,k));
    y_meas = C_sens * X_true(:,k);
    u_obs = [ zeros(m,1); y_meas ];
    X_est(:,k+1) = X_est(:,k) + dt_obs * (A_obs * X_est(:,k) + B_obs * u_obs);
end

err_norm = vecnorm(X_true - X_est, 2, 1);

fprintf('\n=== Observer Convergence (Aggressive Gains) ===\n');
fprintf('Initial estimation error norm = %8.4e\n', err_norm(1));
check_times = [0.05, 0.1, 0.2, 0.5, 1];
for tau = check_times
    [~, idx] = min(abs(t_obs - tau));
    fprintf('  ||e(t=%.2f)|| = %8.4e\n', tau, err_norm(idx));
end
idx_thresh = find(err_norm < 1e-3, 1);
if isempty(idx_thresh)
    fprintf('Error did not drop below 1e-3 within %.2f s\n', Tend_obs);
else
    fprintf('Error dropped below 1e-3 at t = %.2f s\n', t_obs(idx_thresh));
end

%% 9) Combined Closed‐Loop w/ Observer (linear, nonzero init & noise)
Tend_cl = 10; dt_cl = 0.01; t_cl = (0:dt_cl:Tend_cl)';
nt_cl = numel(t_cl);

X_pl = zeros(n, nt_cl);
X_es = zeros(n, nt_cl);
z_int = zeros(ni, nt_cl);

X_pl(:,1) = zeros(n,1);
X_es(:,1) = 5 * ones(n,1);

ref = zeros(ni, nt_cl);
ref(3,:) = 0.1;

for k = 1:nt_cl-1
    u_cmd = -Kx * X_es(:,k) - Ki * z_int(:,k);
    X_pl(:,k+1) = X_pl(:,k) + dt_cl * (A_aug * X_pl(:,k) + B_aug * u_cmd);
    y_m = C_sens * X_pl(:,k) + sqrt(Rn_vals(:)).*randn(p,1);
    u_obs = [ u_cmd; y_m ];
    X_es(:,k+1) = X_es(:,k) + dt_cl * (A_obs * X_es(:,k) + B_obs * u_obs);
    e_int = ref(:,k) - C_int * X_es(:,k);
    z_int(:,k+1) = z_int(:,k) + dt_cl * e_int;
end

track_err = abs((C_int(3,:) * X_pl) - ref(3,:));

err_ss_cl = track_err(end);
est_err_norm = vecnorm(X_pl - X_es, 2, 1);
max_est_err = max(est_err_norm);

idx_tiny = find(track_err < 1e-2, 1);
if isempty(idx_tiny)
    t_tiny = NaN;
else
    t_tiny = t_cl(idx_tiny);
end

fprintf('\n=== Closed‐Loop with Observer (Nonzero Init & Noise) ===\n');
fprintf('Steady‐state z‐error         = %8.4e\n', err_ss_cl);
fprintf('Max estimation error norm    = %8.4e\n', max_est_err);
if isnan(t_tiny)
    fprintf('Tracking error < 1e-2 never reached within %.2f s\n', Tend_cl);
else
    fprintf('Tracking error < 1e-2 at t = %.2f s\n', t_tiny);
end

fprintf('\n=== End of Updated Tests ===\n');
