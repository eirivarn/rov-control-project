% Builds LTI plant, LQR gains, observer gains, and actuator/mixer params

%% 1) Actuator + geometry parameters
tau_act = 0.1;                      % thruster time constant [s]
K      = 40;                       % thrust coefficient [N per unit command]
F_max  = 30;                       % thrust saturation limit [N]
Tmix   = buildMixingMatrix();  % 6×8 thruster‐mixing matrix

%% 2) Augmented 6‐DOF + actuator plant
[sys_aug, A_aug, B_aug, C_aug, D_aug] = rovWithActuators();
n = size(A_aug,1);    % =20
m = size(B_aug,2);    % = 8
ni = 6;               % # of integrators (first 6 outputs)

A_h = A_aug(1:12,   1:12);         % 12×12
B_h = B_aug(1:12, :    );          % 12×8
C_h = C_aug(1:12, :    );          % 12×20
D_h = zeros(12, size(B_h,2));      % 12×8

%% 3) LQR with integrators
Qx = eye(n);          % state‐cost
Qu = 0.1*eye(m);      % input‐cost
Qi = 10;              % integrator‐cost

[Kx, Ki] = designHoverController(Qx, Qu, Qi);
%   – Kx is (8×20), Ki is (8×6)

%% 4) Kalman‐filter (LQE) observer on hydro states
meas_idx = 1:12;                  % measure only the 12 hydro states
C_real   = C_aug(meas_idx, :);    % 12×20

Qn = 1e-2 * eye(n);
Rn = 1e-1 * eye(numel(meas_idx));
G  = eye(n);

L_aug = lqe(A_aug, G, C_real, Qn, Rn);  % 20×12

% Partition L_aug for manual‐wiring Observer blocks
L_hydro    = L_aug(1:12, :);    % 12×12 gain into hydro‐state update
L_actuator = L_aug(13:20,:);    % 8×12 gain into actuator‐state update

%% 5) (optional) Build full Observer SS matrices
A_obs = A_aug - L_aug*C_aug;      
B_obs = [B_aug, L_aug];           
C_obs = eye(n);                   
D_obs = zeros(n, size(B_obs,2));  

%% 6) For closed‐loop linear analysis (no saturation)
C_int = C_aug(1:ni, :);           % 6×20
Acl = [ A_aug - B_aug*Kx,    -B_aug*Ki;
       -C_int,               zeros(ni) ];

Br  = [ zeros(n,ni);    eye(ni) ];
Bd  = [ eye(n),       zeros(n,ni);
        zeros(ni,n),  zeros(ni,ni) ];

Ccl = [ eye(n), zeros(n,ni) ];
Dcl = zeros(n, size(Br,2)+size(Bd,2));

sys_cl = ss(Acl, [Br, Bd], Ccl, Dcl);

%% 7) Export to workspace
fprintf('Workspace setup complete. You can now open/run your Simulink model.\n');
