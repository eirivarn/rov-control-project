%% 1. Load system and design nominal LQR controller
[sys, A, B, C, D] = rovHoverModel();
Qx = eye(12);
Qu = 0.1 * eye(6);
Qi = 10;
[Kx, Ki] = designHoverController(Qx, Qu, Qi);

%% 1.1 Build a 2-D gain schedule over (φ,θ) and flatten it for Simulink 1D Lookup Table

% Define the attitude grid
phi_grid   = linspace(-pi/6, +pi/6, 5);    % –30° … +30°
theta_grid = linspace(-pi/6, +pi/6, 5);    % –30° … +30°
N_phi      = numel(phi_grid);
N_theta    = numel(theta_grid);

% Preallocate storage
K_table         = zeros(6, 12, N_phi, N_theta);             % Full gain grid
K_table_flat    = zeros(N_phi * N_theta, 72);               % Flattened: [25 x 72]
lookup_phi_vals = zeros(N_phi * N_theta, 1);                % 25 φ values
lookup_theta_vals = zeros(N_phi * N_theta, 1);              % 25 θ values

idx = 1;
for i = 1:N_phi
  for j = 1:N_theta
    [A_ij, B_ij] = linearizeROV(phi_grid(i), theta_grid(j));
    K_ij = lqr(A_ij, B_ij, Qx, Qu);                          % [6×12]
    
    K_table(:,:,i,j)   = K_ij;
    K_table_flat(idx,:) = reshape(K_ij, 1, []);             % [1×72]
    
    lookup_phi_vals(idx)   = phi_grid(i);                   % Remember φ
    lookup_theta_vals(idx) = theta_grid(j);                 % Remember θ
    idx = idx + 1;
  end
end

% Save for Simulink
schedData.phi_grid         = phi_grid;
schedData.theta_grid       = theta_grid;
schedData.K_table          = K_table;          % Optional: full grid
schedData.K_table_flat     = K_table_flat;     % [25 x 72]
schedData.lookup_phi_vals  = lookup_phi_vals;  % [25 x 1]
schedData.lookup_theta_vals= lookup_theta_vals;% [25 x 1]

save('K_schedule.mat', 'schedData');

%% 1.5 Observer design using Kalman filter (LQE)
meas_idx = 1:12;                       % assume we measure all 12 states
C_full   = eye(12);
C_real   = C_full(meas_idx,:);        % picks out rows 1:12 of I₁₂

Qn = 1e-2 * eye(12);                  % process noise covariance
Rn = 1e-1 * eye(numel(meas_idx));     % measurement noise covariance
G  = eye(12);                         % process noise on all states
L  = lqe(A, G, C_real, Qn, Rn);       % Kalman observer gain (12×12)

%% 2. Closed-loop augmented system with [reference r (6), disturbance d (6)]
C_integration = C(1:6,:);
n     = size(A,1);    % 12
ni    = size(C_integration,1); % 6
m     = size(B,2);    % 6

Acl    = [A - B*Kx,     -B*Ki;
          -C_integration, zeros(ni)];
B_r    = [zeros(n, ni); eye(ni)];
B_d    = [eye(n,6);     zeros(ni,6)];
B_comb = [B_r, B_d];                  % inputs = [r(6); d(6)]
Ccl    = [eye(n), zeros(n, ni)];
Dcl    = zeros(n, size(B_comb,2));

sys_cl = ss(Acl, B_comb, Ccl, Dcl);
