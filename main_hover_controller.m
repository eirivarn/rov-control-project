%% 1. Load augmented system and design nominal LQR controller
[sys_aug, A_aug, B_aug, C_aug, D_aug] = rovWithActuators();

n  = size(A_aug,1);    % =20
m  = size(B_aug,2);    % = 8
ni = 6;                % still integrating the first 6 outputs

Qx = eye(n);
Qu = 0.1 * eye(m);
Qi = 10;

[Kx, Ki] = designHoverController(Qx, Qu, Qi);
%   – Kx is (8×20), Ki is (8×6)


%% 1.1  Build a 2-D gain schedule over (φ,θ)
%{
N_phi   = 5;
N_theta = 5;
phi_min   = -pi/6;
phi_max   = +pi/6;
theta_min = -pi/6;
theta_max = +pi/6;

phi_grid   = linspace(phi_min, phi_max, N_phi);
theta_grid = linspace(theta_min, theta_max, N_theta);

bin_w_φ = (phi_max   - phi_min)   / (N_phi-1);
bin_w_θ = (theta_max - theta_min) / (N_theta-1);

% Pre-allocate: full feedback and integrator gains
Kx_table_flat = zeros(N_phi*N_theta, n*m);  % [25 x 160]
Kx_table_full = zeros(m, n, N_phi, N_theta);
Ki_table_full= repmat(Ki, [1,1,N_phi,N_theta]); % Ki constant

idx = 1;
for i = 1:N_phi
  for j = 1:N_theta
    % 1) linearize the hydro‐only model
    [Ah, Bh] = linearizeROV(phi_grid(i), theta_grid(j));

    % 2) augment with actuator dynamics exactly as in rovWithActuators
    tau   = 0.1;
    K     = 40;
    Tmix  = buildMixingMatrix_Xconfig();
    % A_aug_ij = [Ah,  Bh*Tmix*K;  zeros(8,12), -(1/tau)*eye(8)];
    % B_aug_ij = [zeros(12,8); (1/tau)*eye(8)];
    A_aug_ij = [ Ah,  Bh*Tmix*K; 
                 zeros(8,12), -(1/tau)*eye(8) ];
    B_aug_ij = [ zeros(12,8); 
                 (1/tau)*eye(8) ];

    % 3) compute LQR on the augmented slice
    Kfull_ij = lqr(A_aug_ij, B_aug_ij, Qx, Qu);  % 8×20

    % 4) store
    Kx_table_full(:,:,i,j) = Kfull_ij;           % 8×20×5×5
    Kx_table_flat(idx,:)   = reshape(Kfull_ij,1,[]);

    idx = idx + 1;
  end
end

schedData = struct(...
  'Kx_table_flat',   Kx_table_flat,...
  'Kx_table_full',   Kx_table_full,...
  'Ki_table_full',   Ki_table_full,...
  'phi_min',         phi_min,...
  'theta_min',       theta_min,...
  'bin_width_phi',   bin_w_φ,...
  'bin_width_theta', bin_w_θ,...
  'N_phi',           N_phi,...
  'N_theta',         N_theta,...
  'phi_grid',        phi_grid,...
  'theta_grid',      theta_grid);

save('K_schedule.mat','schedData');
%}

%% 1.5  Observer design using Kalman filter (LQE)
meas_idx = 1:12;                     % still measuring the 12 hydro states
C_real   = C_aug(meas_idx, :);      % 12×20 measurement matrix

Qn = 1e-2 * eye(n);
Rn = 1e-1 * eye(numel(meas_idx));
G  = eye(n);

L_aug = lqe(A_aug, G, C_real, Qn, Rn);  % 20×12

%% 2. Closed-loop augmented system with integrators
% Integrate the first 6 outputs (e.g. position/orientation errors)
C_int = C_aug(1:ni, :);    % 6×20

% Build Acl with [x_aug; xi] states:
%   top‐block:   A_aug−B_aug*Kx    |  −B_aug*Ki
%   bottom‐blk:  −C_int            |   zeros(6×6)
Acl = [ ...
  A_aug - B_aug*Kx,    -B_aug*Ki; 
  -C_int,              zeros(ni)  ...
];

% Reference & disturbance inputs (if you need them)
Br  = [ zeros(n,ni);    eye(ni) ];
Bd  = [ eye(n),         zeros(n,ni);
        zeros(ni,n),    zeros(ni,ni) ];

Ccl = [ eye(n), zeros(n,ni) ];
Dcl = zeros(n, size(Br,2)+size(Bd,2));

sys_cl = ss(Acl, [Br, Bd], Ccl, Dcl);