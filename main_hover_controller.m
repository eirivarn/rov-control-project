% Builds LTI plant, LQR gains, observer gains (with realistic sensors),
% and actuator/mixer params for the BlueROV2,
% with toggleable onboard measurements.

%% 0) User toggles: enable or disable each sensor here
usePressure     = true;   % depth z
useIMU          = true;   % roll φ, pitch θ
useMagnetometer = true;   % yaw ψ
useDVL          = true;   % surge u, sway v
useSBL          = true;   % position x, y (USBL/SBL)

%% 1) Actuator + geometry parameters
tau_act = 0.1;                      % thruster time constant [s]
K       = 40;                       % thrust coefficient [N per unit command]
Tmix    = buildMixingMatrix();      % 6×8 thruster-mixing matrix

%% 2) Augmented 6-DOF + actuator plant
[sys_aug, A_aug, B_aug, C_aug, D_aug] = rovWithActuators();
n  = size(A_aug,1);    % =20 total states
m  = size(B_aug,2);    % =8 thruster inputs
ni = 6;                % # of integrators (first 6 outputs)

%% 3) LQR with integrators
Qx = eye(n);          % state-cost
Qu = 0.1*eye(m);      % input-cost
Qi = 10;              % integrator-cost
[Kx, Ki] = designHoverController(Qx, Qu, Qi);

%% 4) Sensor selection & measurement matrices
% We define for each sensor its state-index(es) and noise variance:
%   Pressure → z (3)        var = 1e-5
%   IMU      → φ,θ (4,5)    var = 1e-4 each
%   Magnetom.→ ψ (6)        var = 1e-3
%   DVL      → u,v (7,8)    var = 1e-2 each
%   SBL      → x,y (1,2)    var = 1e-1 each

measIdx  = [];
Rn_vals  = [];

% 1) x & y (USBL/SBL)
if useSBL
    measIdx(end+1:end+2) = [1, 2];      % x=1, y=2
    Rn_vals(end+1:end+2) = [1e-1, 1e-1];
end

% 2) z (pressure)
if usePressure
    measIdx(end+1) = 3;                 % z=3
    Rn_vals(end+1) = 1e-5;
end

% 3) φ & θ (IMU)
if useIMU
    measIdx(end+1:end+2) = [4, 5];      % φ=4, θ=5
    Rn_vals(end+1:end+2) = [1e-4, 1e-4];
end

% 4) ψ (magnetometer)
if useMagnetometer
    measIdx(end+1) = 6;                 % ψ=6
    Rn_vals(end+1) = 1e-3;
end

% 5) u & v (DVL)
if useDVL
    measIdx(end+1:end+2) = [7, 8];      % u=7, v=8
    Rn_vals(end+1:end+2) = [1e-2, 1e-2];
end

% now build C_sens, D_sens and LQE as before
p = numel(measIdx);
C_sens = zeros(p, n);
for i = 1:p
    C_sens(i, measIdx(i)) = 1;
end
D_sens = zeros(p, m);

Qn = 1e-2*eye(n);
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

fprintf('Workspace ready. Measurements used (state indices): %s\n', ...
        strjoin(string(measIdx), ','));
