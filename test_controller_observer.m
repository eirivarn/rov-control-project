% test_controller_observer.m
% Tests LQR controller and Kalman-filter-style observer.
% Allows tuning of Qx, Qu, Qi for LQR and Qn, Rn for the observer.
% Sensor noise variances (Rn_vals_tune_...) are initialized to match report values.
clear; clc; close all
%% 0) User-Defined Tuning Parameters
% --- LQR Controller Tuning (Initialized to your "best results") ---
Qx_tune = diag([ones(1,6)*2.5, zeros(1,6), zeros(1,8)]);
Qu_tune_scalar = 0.05;
Qi_tune = 25;
% --- Observer (LQE) Tuning ---
% Qn_tune_scalar: Process noise covariance scalar. Qn = Qn_tune_scalar * eye(n).
Qn_tune_scalar = 1e-2; % Kept from previous alignment
% Rn_vals_tune_SENSOR: Individual measurement noise variances (sigma^2) for each sensor.
% Initialized to match report values.
Rn_vals_tune_SBL       = [1.3e-3, 1.3e-3]; % Aligned with report
Rn_vals_tune_Pressure  = 1.5e-5;         % Aligned with report
Rn_vals_tune_IMU       = [1e-5, 1e-5];     % Aligned with report
Rn_vals_tune_Mag       = 1e-5;           % Aligned with report
Rn_vals_tune_DVL       = [2.5e-5, 2.5e-5]; % Aligned with report
% --- Sensor Selection (used in Observer design, Section 7) ---
usePressure_obs     = true;  % z
useIMU_obs          = true;  % phi, theta
useMagnetometer_obs = true;  % psi
useDVL_obs          = true;  % u, v
useSBL_obs          = true;  % x, y
fprintf('=== Using Tunable Parameters ===\n');
fprintf('LQR: Qi_tune = %g, Qu_tune_scalar = %g (for Qu=Qu_scalar*eye(m))\n', Qi_tune, Qu_tune_scalar);
fprintf(' Using Qx_tune as defined. Checksum: %g\n', sum(Qx_tune(:)));
fprintf('Observer: Qn_tune_scalar = %g (for Qn=Qn_scalar*eye(n))\n', Qn_tune_scalar);
fprintf(' Rn_vals (aligned with report) for SBL=[%g,%g], Pressure=%g, IMU=[%g,%g], Mag=%g, DVL=[%g,%g]\n', ...
    Rn_vals_tune_SBL, Rn_vals_tune_Pressure, Rn_vals_tune_IMU, Rn_vals_tune_Mag, Rn_vals_tune_DVL);
%% 1) Build the 20‐state “hydro + actuator” model
[sys_aug, A_aug, B_aug, C_aug, ~, Tmix] = rovWithActuators();
n  = size(A_aug,1);
m  = size(B_aug,2);
ni = 6;
% Validate Qx_tune dimensions if it was user-defined as a matrix
if ~all(size(Qx_tune) == [n,n])
    warning('User-defined Qx_tune has incorrect dimensions for n=%d. Check definition.', n);
    % Potentially halt or revert to a default if critical, but for now, just warn.
end
fprintf('\n=== Model Dimensions ===\n');
fprintf('A_aug: %d×%d, B_aug: %d×%d, C_aug: %d×%d, Tmix: %d×%d\n', ...
    size(A_aug,1), size(A_aug,2), size(B_aug,1), size(B_aug,2), ...
    size(C_aug,1), size(C_aug,2), size(Tmix,1), size(Tmix,2));
%% 2) Mixing Matrix Rank (No changes, independent of tuning)
rank_Tmix = rank(Tmix);
sv_Tmix   = svd(Tmix);
fprintf('\n=== Mixing Matrix Tmix ===\n');
fprintf('rank(Tmix) = %d (expected 6)\n', rank_Tmix);
fprintf('singular values of Tmix = [%s]\n', sprintf('%6.3f ', sv_Tmix));
%% 3) Instantaneous Reachability & Observability (Standard Method)
% This section checks the instantaneous properties. A rank deficiency here
% is expected for a physical system with integrators (like position) and
% unmeasured states (like actuator forces). The key is that the system
% is "stabilizable" and "detectable", meaning all unstable modes can be
% controlled and observed, which is sufficient for the controller to work.
C_hydro_test = [eye(12), zeros(12, n-12)];
Wc_full  = ctrb(A_aug, B_aug);
rWc_full = rank(Wc_full);
svWc_full = svd(Wc_full);
Wo_full  = obsv(A_aug, C_hydro_test);
rWo_full = rank(Wo_full);
svWo_full = svd(Wo_full);
fprintf('\n=== Instantaneous Gramian Checks (ctrb/obsv) ===\n');
fprintf('Controllability: rank = %d/%d,  min(sv) = %8.4e\n', rWc_full, n, min(svWc_full(svWc_full > 1e-9)));
fprintf('Observability (of hydro states): rank = %d/%d,  min(sv) = %8.4e\n', rWo_full, n, min(svWo_full(svWo_full > 1e-9)));
%% 4) Design LQR + integrators (Using tunable Qx, Qu, Qi)
Qx_LQR = Qx_tune;
Qu_LQR = Qu_tune_scalar * eye(m);
Qi_LQR = Qi_tune;
[Kx, Ki] = designHoverController(Qx_LQR, Qu_LQR, Qi_LQR);
if size(C_aug,1) >= ni
    C_int = C_aug(1:ni,:);
else
    C_int = [eye(ni), zeros(ni, n-ni)];
end
Acl = [ A_aug - B_aug*Kx,   -B_aug*Ki;
        -C_int,             zeros(ni, ni) ];
fprintf('\n=== Controller Gains (Using Tuned Qx, Qu, Qi) ===\n');
fprintf('Kx size: %d×%d,   Ki size: %d×%d\n', size(Kx), size(Ki));
fprintf('Used: Qi=%g, Qu_scalar=%g. Qx checksum: %g\n', Qi_LQR, Qu_tune_scalar, sum(Qx_LQR(:)));
%% 5) Closed‐Loop Eigenvalues & Damping (Results depend on tuned gains)
lambda_cl = eig(Acl);
fprintf('\n=== Closed‐Loop Eigenvalues ===\n');
for i = 1:length(lambda_cl)
    fprintf('  %2d: Re = %+8.4e,  Im = %+8.4e\n', i, real(lambda_cl(i)), imag(lambda_cl(i)));
end
maxRe_cl = max(real(lambda_cl));
if maxRe_cl < -1e-9
    fprintf('Max Re(eig(Acl)) = %+8.4e → stable\n', maxRe_cl);
else
    fprintf('Max Re(eig(Acl)) = %+8.4e → UNSTABLE or marginal\n', maxRe_cl);
end
if any(real(lambda_cl) > 1e-9)
    fprintf('System is unstable, damping ratios might not be meaningful for unstable modes.\n');
    Acl_ss = ss(Acl, [], eye(size(Acl,1)), []);
else
    Acl_ss = ss(Acl, [], eye(size(Acl,1)), []);
end
[wn_cl, zeta_cl, ~] = damp(Acl_ss);
fprintf('\nNatural frequencies (rad/s) & damping ratios:\n');
valid_poles_idx = wn_cl > 1e-6;
wn_cl_filtered = wn_cl(valid_poles_idx);
zeta_cl_filtered = zeta_cl(valid_poles_idx);
poles_filtered = lambda_cl(valid_poles_idx);
for i = 1:length(wn_cl_filtered)
    fprintf('  Mode %2d: wn = %6.3f,  zeta = %5.3f (Pole: %+8.4e %+8.4ej)\n', ...
            i, wn_cl_filtered(i), zeta_cl_filtered(i), real(poles_filtered(i)), imag(poles_filtered(i)));
end
%% 6) Linear Step‐Response Performance (per DOF) (Results depend on tuned gains)
Bcl_ref = [ zeros(n, ni); eye(ni) ];
Ccl_plant_outputs = [eye(ni), zeros(ni, n-ni)];
Ccl_full_state = [Ccl_plant_outputs, zeros(ni, ni)];
Dcl = zeros(ni, ni);
sys_cl = ss(Acl, Bcl_ref, Ccl_full_state, Dcl);
fprintf('\n=== Linear Step‐Response Metrics ===\n');
ref_mag = 0.1;
Tend = 15; dt = 0.01; t = (0:dt:Tend)';
tol_pct = 0.02;
for i = 1:ni
    Uref_vec = zeros(numel(t), ni);
    Uref_vec(:, i) = ref_mag;
    Y = lsim(sys_cl, Uref_vec, t);
    y_i = Y(:, i);
    y_ss = y_i(end);
    err_ss = abs(ref_mag - y_ss);
    if ref_mag > 0
        y_peak = max(y_i);
        overshoot = max(0, (y_peak / ref_mag - 1) * 100);
    else
        y_peak = min(y_i);
        overshoot = max(0, (y_peak / ref_mag - 1) * 100);
    end
    settled_value_min = ref_mag * (1 - tol_pct);
    settled_value_max = ref_mag * (1 + tol_pct);
    t_settle = NaN;
    outside_band_indices = find(y_i < settled_value_min | y_i > settled_value_max);
    if isempty(outside_band_indices)
        if max(abs(y_i - ref_mag)) <= tol_pct * abs(ref_mag)
             t_settle = t(1);
        end
    else
        last_outside_idx = outside_band_indices(end);
        if last_outside_idx < length(t)
            if all(y_i(last_outside_idx+1:end) >= settled_value_min & y_i(last_outside_idx+1:end) <= settled_value_max)
                t_settle = t(last_outside_idx + 1);
            end
        end
    end
     if isnan(t_settle) && max(abs(y_i(end) - ref_mag)) <= tol_pct * abs(ref_mag)
        for j_settle = length(t) : -1 : 1
            if all(y_i(j_settle:end) >= settled_value_min & y_i(j_settle:end) <= settled_value_max)
                t_settle = t(j_settle);
            else
                break;
            end
        end
    end
    fprintf('DOF %2d: ss error = %8.4e,  overshoot = %5.2f%%,  settling time = %5.2f s\n', ...
            i, err_ss, overshoot, t_settle);
end
%% 7) Design Observer (Kalman/Luenberger) using tunable Qn and report-aligned Rn_vals
measIdx = [];
Rn_vals_active = [];
if useSBL_obs
    measIdx(end+1:end+2) = [1, 2];
    Rn_vals_active(end+1:end+2) = Rn_vals_tune_SBL;
end
if usePressure_obs
    measIdx(end+1) = 3;
    Rn_vals_active(end+1) = Rn_vals_tune_Pressure;
end
if useIMU_obs
    measIdx(end+1:end+2) = [4, 5];
    Rn_vals_active(end+1:end+2) = Rn_vals_tune_IMU;
end
if useMagnetometer_obs
    measIdx(end+1) = 6;
    Rn_vals_active(end+1) = Rn_vals_tune_Mag;
end
if useDVL_obs
    measIdx(end+1:end+2) = [7, 8];
    Rn_vals_active(end+1:end+2) = Rn_vals_tune_DVL;
end
p_num_meas = numel(measIdx);
if p_num_meas == 0
    error('No sensors selected for the observer. Halting.');
end
C_sens = zeros(p_num_meas, n);
for k_sens = 1:p_num_meas
    C_sens(k_sens, measIdx(k_sens)) = 1;
end
Qn_obs = Qn_tune_scalar * eye(n);
Rn_obs = diag(Rn_vals_active);
G_proc_noise = eye(n);
L_sens = lqe(A_aug, G_proc_noise, C_sens, Qn_obs, Rn_obs);
L_hydro    = L_sens(1:12, :);
L_actuator = L_sens(13:20, :);
A_obs = A_aug - L_sens * C_sens;
B_obs = [ B_aug,   L_sens ];
C_obs = eye(n);
D_obs = zeros(n, size(B_obs,2));
fprintf('\n=== Observer Design (Using Tuned Qn and Report-Aligned Rn) ===\n');
fprintf('Active sensors (measIdx): [%s]\n', num2str(measIdx));
fprintf('C_sens size: %d×%d,   Rn_obs (diag(Rn_vals_active)) size: %d×%d\n', size(C_sens), size(Rn_obs));
fprintf('L_sens size: %d×%d. Used Qn_scalar=%g.\n', size(L_sens), Qn_tune_scalar);
Wo_obs = obsv(A_aug, C_sens);
rWo_obs = rank(Wo_obs);
svWo_obs_nz = svd(Wo_obs);
svWo_obs_nz = svWo_obs_nz(svWo_obs_nz > 1e-9);
min_svWo_obs = 0;
if ~isempty(svWo_obs_nz)
    min_svWo_obs = min(svWo_obs_nz);
end
fprintf('\nObservability (with selected sensors, instantaneous): rank = %d/%d, min(sv) = %8.4e\n', rWo_obs, n, min_svWo_obs);
eig_obs = eig(A_obs);
slowest_obs_pole = max(real(eig_obs));
fprintf('Slowest observer pole (real part) = %+8.4e\n', slowest_obs_pole);
%% 8) Observer Convergence (Results depend on tuned observer gains)
Tend_obs = 2; dt_obs = 0.002; t_obs = (0:dt_obs:Tend_obs)';
nt_obs = numel(t_obs);
X_true = zeros(n, nt_obs);
X_est  = zeros(n, nt_obs);
X_true(:,1) = zeros(n,1);
X_est(:,1)  = 2 * ones(n,1);
sys_plant_obs_test = ss(A_aug, [], C_sens, []);
sys_estimator_obs_test = ss(A_obs, B_obs, eye(n), []);
U_plant_zero = zeros(nt_obs, 0);
[Y_true_meas, ~, X_true_path] = lsim(sys_plant_obs_test, U_plant_zero, t_obs, X_true(:,1));
X_true = X_true_path';
U_observer_input = [zeros(nt_obs, m), Y_true_meas];
[~, ~, X_est_path] = lsim(sys_estimator_obs_test, U_observer_input, t_obs, X_est(:,1));
X_est = X_est_path';
err_norm = vecnorm(X_true - X_est, 2, 1);
fprintf('\n=== Observer Convergence (Tuned Gains) ===\n');
fprintf('Initial estimation error norm = %8.4e\n', err_norm(1));
check_times = [0.1, 0.2, 0.5, 1.0, 2.0];
for tau_check = check_times
    if tau_check <= Tend_obs
        [~, idx_check] = min(abs(t_obs - tau_check));
        fprintf('  ||e(t=%.2f)|| = %8.4e\n', tau_check, err_norm(idx_check));
    end
end
idx_thresh = find(err_norm < 1e-3, 1);
if isempty(idx_thresh)
    fprintf('Error did not drop below 1e-3 within %.2f s. Min error norm: %8.4e at t=%.2f s\n', ...
        Tend_obs, min(err_norm), t_obs(err_norm == min(err_norm)));
else
    fprintf('Error dropped below 1e-3 at t = %.3f s\n', t_obs(idx_thresh));
end
%% 9) Combined Closed‐Loop w/ Observer (Results depend on all tuned gains)
Tend_cl = 15; dt_cl = 0.01; t_cl = (0:dt_cl:Tend_cl)';
nt_cl = numel(t_cl);
X_plant_state = zeros(n, nt_cl);
X_observer_state = zeros(n, nt_cl);
integrator_state = zeros(ni, nt_cl);
X_plant_state(:,1) = zeros(n,1);
X_observer_state(:,1) = 1*ones(n,1);
integrator_state(:,1) = zeros(ni,1);
reference_signal = zeros(ni, nt_cl);
reference_signal(3,:) = 0.1;
for k_sim = 1:nt_cl-1
    u_cmd = -Kx * X_observer_state(:,k_sim) - Ki * integrator_state(:,k_sim);
    dX_plant = A_aug * X_plant_state(:,k_sim) + B_aug * u_cmd;
    X_plant_state(:,k_sim+1) = X_plant_state(:,k_sim) + dt_cl * dX_plant;
    measurement_noise = sqrt(Rn_vals_active(:)) .* randn(p_num_meas, 1);
    y_measured = C_sens * X_plant_state(:,k_sim) + measurement_noise;
    observer_inputs = [u_cmd; y_measured];
    dX_observer = A_obs * X_observer_state(:,k_sim) + B_obs * observer_inputs;
    X_observer_state(:,k_sim+1) = X_observer_state(:,k_sim) + dt_cl * dX_observer;
    tracking_error_integrator = reference_signal(:,k_sim) - C_int * X_observer_state(:,k_sim);
    integrator_state(:,k_sim+1) = integrator_state(:,k_sim) + dt_cl * tracking_error_integrator;
end
actual_z_trajectory = C_int(3,:) * X_plant_state;
target_z_trajectory = reference_signal(3,:);
tracking_error_z = abs(actual_z_trajectory - target_z_trajectory);
steady_state_tracking_error_z = tracking_error_z(end);
estimation_error_norm_trajectory = vecnorm(X_plant_state - X_observer_state, 2, 1);
max_estimation_error_norm = max(estimation_error_norm_trajectory);
idx_tracking_threshold_z = find(tracking_error_z < 0.01, 1);
time_to_reach_tracking_threshold_z = NaN;
if ~isempty(idx_tracking_threshold_z)
    time_to_reach_tracking_threshold_z = t_cl(idx_tracking_threshold_z);
end
fprintf('\n=== Closed‐Loop with Observer (Tuned Params, Nonzero Init & Noise) ===\n');
fprintf('Target: 0.1m step in z (DOF 3)\n');
fprintf('Initial plant state: zeros, Initial observer estimate: %g*ones\n', X_observer_state(1,1) / (ones(1,1)+eps) );
fprintf('Steady‐state z‐tracking error = %8.4e m\n', steady_state_tracking_error_z);
fprintf('Max estimation error norm     = %8.4e\n', max_estimation_error_norm);
if isnan(time_to_reach_tracking_threshold_z)
    fprintf('z-tracking error < 0.01m (1cm) never reached within %.2f s. Final error: %.4e m\n', ...
        Tend_cl, tracking_error_z(end));
else
    fprintf('z-tracking error < 0.01m (1cm) at t = %.2f s\n', time_to_reach_tracking_threshold_z);
end

fprintf('\n=== End of Tunable Parameter Tests ===\n');
