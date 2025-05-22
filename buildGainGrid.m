% Precompute a 6×6 grid (36 points) of LQR+integrator gains.
deg = pi/180;

% 1) grid of roll φ and pitch θ trims
phi_vals   = linspace(-30, 30, 6)*deg;   
theta_vals = linspace(-30, 30, 6)*deg;   

% 2) base linearization at zero (just to get dimensions)
[sys0, A0, B0] = rovWithActuators();  % ensure this returns A0,B0
n  = size(A0,1);
m  = size(B0,2);
ni = 6;

% 3) allocate storage
Kx_all = zeros(m, n, numel(phi_vals)*numel(theta_vals));
Ki_all = zeros(m, ni, numel(phi_vals)*numel(theta_vals));
grid_pts = zeros(numel(phi_vals)*numel(theta_vals), 2);

% 4) cost settings
Qx = eye(n);
Qu = 0.1*eye(m);
Qi = 10;

% 5) loop over trims
idx = 0;
for i = 1:numel(phi_vals)
  for j = 1:numel(theta_vals)
    idx = idx + 1;
    phi0   = phi_vals(i);
    theta0 = theta_vals(j);

    % --------------------------------------------------------------------
    % LINEARIZE YOUR PLANT AT (phi0,theta0)
    % --------------------------------------------------------------------
    % You need a function that takes (phi0,theta0) and returns (A,B).
    % If you have a Simulink model 'rov_kinematics_level3', you could do:
    %
    %   op = operpoint('rov_kinematics_level3');
    %   op.States(4).x = phi0;    % set phi
    %   op.States(5).x = theta0;  % set theta
    %   io = getlinio('rov_kinematics_level3');
    %   syslin = linearize('rov_kinematics_level3', op, io);
    %   [A,~,B,~] = ssdata(syslin);
    %
    % For simplicity, here we just reuse the zero‐trim A0,B0:
    A = A0;
    B = B0;
    % --------------------------------------------------------------------

    % store grid point
    grid_pts(idx,:) = [phi0, theta0];

    % design LQR+integrator at this trim
    [Kx, Ki] = designHoverControllerAtTrim(A, B, Qx, Qu, Qi);

    % save
    Kx_all(:,:,idx) = Kx;
    Ki_all(:,:,idx) = Ki;
  end
end

% 6) save to .mat
save('gainGrid36.mat', 'phi_vals', 'theta_vals', 'grid_pts', 'Kx_all', 'Ki_all');
