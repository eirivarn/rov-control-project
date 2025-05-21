%% GUI-STYLE TOGGLES
enable_currents   = true;
enable_waves      = true;
add_sensor_noise  = true;
use_observer      = true;   % ← New: toggle to enable observer

%% 1. Load system and design controller
[sys, A, B, C, D] = rovHoverModel();
Qx = eye(12);
Qu = 0.1 * eye(6);
Qi = 10;
[Kx, Ki] = designHoverController(Qx, Qu, Qi);

%% 1.5 Observer design using Kalman filter (LQE)
meas_idx = [1:12];C_real   = eye(12);
C_real   = C_real(meas_idx, :);      % Measurement matrix 6×12

Qn = 1e-2 * eye(12);                 % Process noise covariance
Rn = 1e-1 * eye(12);                  % Measurement noise covariance
G  = eye(12);                        % Assume process noise on all states
L  = lqe(A, G, C_real, Qn, Rn);      % Kalman observer gain (12×6)

%% 2. Closed-loop system with [reference r (6), disturbance d (6)]
C_integration = C(1:6,:);
n     = size(A,1);  % 12
ni    = size(C_integration,1); % 6
m     = size(B,2);  % 6

Acl    = [A - B*Kx, -B*Ki;
          -C_integration,    zeros(ni)];
B_r    = [zeros(n, ni); eye(ni)];
B_d    = [eye(n,6);     zeros(ni,6)];
B_comb = [B_r, B_d];     % Input: [r(6), d(6)]
Ccl    = [eye(n), zeros(n, ni)];
Dcl    = zeros(n, size(B_comb,2));

sys_cl = ss(Acl, B_comb, Ccl, Dcl);

%% 3. Create inputs: r (step), d (currents + waves)
t = 0:0.1:60;
N = length(t);

r = ones(N, 6) .* (t' >= 5);         % Step ref in position after t = 5

d = zeros(N, 6);                     % Disturbance vector
if enable_currents
    currents = 0.1 * sin(0.2 * t');
    d(:,1:3) = repmat(currents, 1, 3);
end
if enable_waves
    d(:,4:6) = 0.05 * randn(N, 3);
end

u_ext = [r, d];                      % [N×12] input

%% 4. Simulate system response
x0     = randn(12,1) * 0.1;
z0     = zeros(6,1);
x_aug0 = [x0; z0];

[y, t_out, x_out] = lsim(sys_cl, u_ext, t, x_aug0);

x_t = x_out(:, 1:12);                % True states
z_t = x_out(:, 13:end);             % Integrator states


%% 5. Simulate observer (estimate x̂)
if use_observer
    y_meas = x_t(:, meas_idx);
    if add_sensor_noise
        y_meas = y_meas + 0.01 * randn(size(y_meas));
    end

    x_hat = zeros(size(x_t));       % Estimated states
    x_hat(1,:) = zeros(1,12);
    u_t   = zeros(N,6);             % Control input history

    for k = 1:N-1
        dt = t_out(k+1) - t_out(k);

        % Compute control input using current estimate
        u_t(k,:) = - (x_hat(k,:) * Kx') - (z_t(k,:) * Ki');

        % Observer update
        dx_hat = A*x_hat(k,:)' + B*u_t(k,:)' + L*(y_meas(k,:)' - C_real*x_hat(k,:)');
        x_hat(k+1,:) = x_hat(k,:) + dt * dx_hat';
    end

    % Final control input (for last step)
    u_t(N,:) = - (x_hat(N,:) * Kx') - (z_t(N,:) * Ki');

else
    u_t = - (x_t * Kx') - (z_t * Ki');    % Full state used directly
end


%% 6. Plot full state responses
figure;
plot(t_out, x_t, 'LineWidth', 1.2)
xlabel('Time [s]');
ylabel('State Value');
title('ROV Full-State Response');
legend(arrayfun(@(i) sprintf('x_%d', i), 1:12, 'UniformOutput', false), ...
       'Location', 'eastoutside')
grid on;

%% 7. Plot control inputs u(t)
figure;
plot(t_out, u_t, 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Control Input u_i');
title('Control Inputs u(t)');
legend({'u_1','u_2','u_3','u_4','u_5','u_6'}, 'Location', 'eastoutside')
grid on;

%% 8. Plot reference vs actual position states
figure;
for i = 1:6
    subplot(3,2,i)
    plot(t_out, r(:,i), 'k--', t_out, x_t(:,i), 'b-')
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i))
    legend('Reference','Actual'); grid on
end
sgtitle('Position Tracking (States 1–6)')
