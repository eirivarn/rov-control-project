% Builds LTI plant, LQR gains, observer gains (with realistic sensors),
% and actuator/mixer params for the BlueROV2,
% with toggleable onboard measurements.
clear; clc;
p = rovParameters();
%% 0) User toggles: enable or disable each sensor here
usePressure     = true;   % depth z
useIMU          = true;   % roll φ, pitch θ
useMagnetometer = true;   % yaw ψ
useDVL          = true;   % surge u, sway v
useSBL          = true;   % position x, y (USBL/SBL)
%% 1) Actuator + geometry parameters
tau_act = 0.1;                      % thruster time constant [s]
K       = 40;                       % thrust coefficient [N per unit command]
F_max   = 30;                     % thrust saturation limit [N]
Tmix    = buildMixingMatrix();      % 6×8 thruster-mixing matrix
% what thrust each thruster must hold to hover [N]:
f0 = pinv(Tmix) * p.u0;
% convert to normalized command units:
u0_cmd = pinv(Tmix)*rovParameters().u0 / K;
assignin('base','u0_cmd', u0_cmd);
%% 2) Augmented 6-DOF + actuator plant
[sys_aug, A_aug, B_aug, C_aug, D_aug] = rovWithActuators();
n  = size(A_aug,1);    % =20 total states
m  = size(B_aug,2);    % =8 thruster inputs
ni = 6;                % # of integrators (first 6 outputs)
%% 3) LQR with integrators (Using "best results" LQR parameters)
% Qx: Penalizes deviations in hydro (pose, velocity) and actuator states.
Qx = diag([ones(1,6)*2.5, zeros(1,6), zeros(1,8)]); % From "best results"

% Qu: Penalizes thruster effort.
Qu_scalar = 0.05; % From "best results"
Qu = Qu_scalar*eye(m);

% Qi: Penalizes accumulated error for the 6 regulated outputs (x,y,z,phi,theta,psi).
Qi = 25; % From "best results"

[Kx, Ki] = designHoverController(Qx, Qu, Qi);
%% 4) Sensor selection & measurement matrices (Sensor noise aligned with report)
% We define for each sensor its state-index(es) and noise variance (sigma^2):
% Document values:
%   Pressure → z (3)        sigma^2 = 1.5e-5
%   IMU      → φ,θ (4,5)    sigma^2 = 1e-5
%   Magnetom.→ ψ (6)        sigma^2 = 1e-5
%   DVL      → u,v (7,8)    sigma^2 = 2.5e-5
%   SBL      → x,y (1,2)    sigma^2 = 1.3e-3
measIdx  = [];
Rn_vals  = [];
% 1) x & y (USBL/SBL)
if useSBL
    measIdx(end+1:end+2) = [1, 2];      % x=1, y=2
    Rn_vals(end+1:end+2) = [1.3e-3, 1.3e-3]; % Aligned with report
end
% 2) z (pressure)
if usePressure
    measIdx(end+1) = 3;                 % z=3
    Rn_vals(end+1) = 1.5e-5;            % Aligned with report
end
% 3) φ & θ (IMU)
if useIMU
    measIdx(end+1:end+2) = [4, 5];      % φ=4, θ=5
    Rn_vals(end+1:end+2) = [1e-5, 1e-5];   % Aligned with report
end
% 4) ψ (magnetometer)
if useMagnetometer
    measIdx(end+1) = 6;                 % ψ=6
    Rn_vals(end+1) = 1e-5;              % Aligned with report
end
% 5) u & v (DVL)
if useDVL
    measIdx(end+1:end+2) = [7, 8];      % u=7, v=8
    Rn_vals(end+1:end+2) = [2.5e-5, 2.5e-5];% Aligned with report
end

if isempty(measIdx)
    error('No sensors enabled. Observer cannot be designed.');
end
p_num_meas = numel(measIdx);
C_sens = zeros(p_num_meas, n);
for idx_loop = 1:p_num_meas
    C_sens(idx_loop, measIdx(idx_loop)) = 1;
end

D_sens = zeros(p_num_meas, m);

% Qn: Process noise covariance.
Qn_scalar = 1e-2; % Kept from previous alignment
Qn = Qn_scalar*eye(n);

% Rn: Measurement noise covariance.
Rn = diag(Rn_vals);

G  = eye(n); % Assuming process noise enters all states directly

L_sens = lqe(A_aug, G, C_sens, Qn, Rn);
L_hydro    = L_sens(1:12, :);
L_actuator = L_sens(13:20,:);
%% 5) Full observer State-Space
A_obs = A_aug - L_sens * C_sens;
B_obs = [ B_aug,   L_sens ];     % inputs = [thruster_cmds; measurements]
C_obs = eye(n);
D_obs = zeros(n, size(B_obs,2));
sys_observer = ss(A_obs, B_obs, C_obs, D_obs);
%% 6) Closed-loop LTI (analysis only)
if isempty(C_aug) || size(C_aug,1) < ni
    warning('C_aug from rovWithActuators is not as expected or has too few rows. Defaulting C_int.');
    C_int = [eye(ni), zeros(ni, n-ni)];
else
    C_int = C_aug(1:ni, :);
end

Acl   = [ A_aug - B_aug*Kx,  -B_aug*Ki;
         -C_int,             zeros(ni,ni) ];
Br = [ zeros(n,ni); eye(ni) ];
Bd = [ eye(n), zeros(n,ni);
       zeros(ni,n), zeros(ni,ni) ];
Ccl = [ eye(n), zeros(n,ni) ];
Dcl_rows = size(Ccl,1);
Dcl_cols_Br = size(Br,2);
Dcl_cols_Bd = size(Bd,2);
Dcl = zeros(Dcl_rows, Dcl_cols_Br + Dcl_cols_Bd);

sys_cl = ss(Acl, [Br, Bd], Ccl, Dcl);
%% Gain-Scheduling
[phi_vals, theta_vals, psi_vals, grid_pts, Kx_all, Ki_all] = ...
    buildGainGrid(6, 6, 6);
fprintf('Workspace ready. Measurements used (state indices): %s\n', ...
        strjoin(string(measIdx), ','));
fprintf('LQR parameters set to "best results": Qx(1,1)=%g, Qu_scalar=%g, Qi=%g\n', Qx(1,1), Qu_scalar, Qi);
fprintf('Observer Qn_scalar=%g. Rn uses sensor variances aligned with report.\n', Qn_scalar);