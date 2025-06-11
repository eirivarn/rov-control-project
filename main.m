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
F_max   = 40;                       % thrust saturation limit [N]
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

%% 3) LQR with integrators
%  - Qx weights prioritize which states matter most in performance:
%      * High cost on critical states (e.g., depth, roll/pitch) → tighter control
%      * Low or zero cost on actuator/internals → avoid over-penalizing dynamics
%  - Qu sets relative penalty on control effort vs. state error
%  - Qi shapes disturbance rejection and offset elimination:
%      * Larger Qi on axes where zero steady-state error is crucial
%  - Start with heuristic values, then iterate:
%      1. Increase Qx_diag(i) to tighten control on state i
%      2. Adjust Qu_scalar to trade control effort
%      3. Tune Qi_vec to speed bias rejection without oscillation

% Position States (Indices 1-3)
Qx_diag(1:2) = 1.0;  % x, y position cost
Qx_diag(3)   = 1.0;  % z (depth) cost

% Orientation States (Indices 4-6)
Qx_diag(4:5) = 10.0; % roll (phi), pitch (theta) cost
Qx_diag(6)   = 10.0;  % yaw (psi) cost

% Velocity States (Indices 7-12) for Damping
Qx_diag(7:9)   = 10.0;  % u, v, w cost
Qx_diag(10:12) = 10.0; % p, q, r cost

% Actuator States (Indices 13-20)
Qx_diag(13:20) = 0;

% Create the final diagonal matrix from the vector of costs
Qx = diag(Qx_diag);

% Define input and integrator costs
Qu_scalar = 0.01;
Qu = Qu_scalar*eye(m);

%      [ x,   y,    z,   φ,   θ,   ψ ]
Qi_vec = [0.1, 0.1, 0.1, 100.0, 100.0, 100.0];
Qi     = diag(Qi_vec);

% Design the controller with the new Qx
[Kx, Ki] = designHoverController(Qx, Qu, Qi);

%% 4) Sensor selection & measurement matrices
%  - Assign realistic noise variances Rn_vals for each measurement:
%      * Lower variance → trust this sensor more in the Kalman update.
%      * Higher variance → rely more on model predictions.
%  - Qn sets process‐noise covariance: higher → faster response to model mismatch.

%   Pressure → z (3)        
%   IMU      → φ,θ (4,5)    
%   Magnetom.→ ψ (6)        
%   DVL      → u,v (7,8)    
%   SBL      → x,y (1,2)    

measIdx  = [];
Rn_vals  = [];

% 1) x & y (USBL/SBL)
if useSBL
    measIdx(end+1:end+2) = [1, 2];      % x=1, y=2
    Rn_vals(end+1:end+2) = [1.3e-3, 1.3e-3];
end

% 2) z (pressure)
if usePressure
    measIdx(end+1) = 3;                 % z=3
    Rn_vals(end+1) = 1.5e-5;
end

% 3) φ & θ (IMU)
if useIMU
    measIdx(end+1:end+2) = [4, 5];      % φ=4, θ=5
    Rn_vals(end+1:end+2) = [1e-5, 1e-5];
end

% 4) ψ (magnetometer)
if useMagnetometer
    measIdx(end+1) = 6;                 % ψ=6
    Rn_vals(end+1) = 1e-5;
end

% 5) u & v (DVL)
if useDVL
    measIdx(end+1:end+2) = [7, 8];      % u=7, v=8
    Rn_vals(end+1:end+2) = [5e-3, 5e-3];
end

p = numel(measIdx);
C_sens = zeros(p, n);
for i = 1:p
    C_sens(i, measIdx(i)) = 1;
end
D_sens = zeros(p, m);

Qn = 1e-1*eye(n);
Rn = diag(Rn_vals);
G  = eye(n);
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
C_int = C_aug(1:ni, :);

Acl   = [ A_aug - B_aug*Kx,  -B_aug*Ki
         -C_int,             zeros(ni) ];

Br = [ zeros(n,ni); eye(ni) ];

Bd = [ eye(n), zeros(n,ni);
       zeros(ni,n), zeros(ni,ni) ];

Ccl = [ eye(n), zeros(n,ni) ];

Dcl = zeros(n, size(Br,2)+size(Bd,2));

sys_cl = ss(Acl, [Br, Bd], Ccl, Dcl);

%% Gain-Scheduling 
[phi_vals, theta_vals, psi_vals, grid_pts, Kx_all, Ki_all] = ...
    buildGainGrid(6, 6, 6);

fprintf('Workspace ready. Measurements used (state indices): %s\n', ...
        strjoin(string(measIdx), ','));