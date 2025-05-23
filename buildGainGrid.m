deg = pi/180;
Nphi   = 6;   Ntheta = 6;   Npsi = 6;
phi_vals   = linspace(-60, 60, Nphi)*deg;
theta_vals = linspace(-60, 60, Ntheta)*deg;
psi_vals   = linspace(-180,180, Npsi)*deg;

% Pre-allocate
Ngrid = Nphi * Ntheta * Npsi;
% We will discover (n,m) on the first linearization
Kx_all  = [];
Ki_all  = [];
grid_pts = zeros(Ngrid,3);

% Loop & linearize
idx = 0;
for i = 1:Nphi
  for j = 1:Ntheta
    for k = 1:Npsi
      idx = idx + 1;
      phi0   = phi_vals(i);
      theta0 = theta_vals(j);
      psi0   = psi_vals(k);

      % 1) linearize the 6-state simple model at this trim
      [A, B] = linearizeROV(phi0, theta0, psi0);  % A:6x6, B:6x6

      % On first pass, allocate Kx_all & Ki_all and set cost matrices
      if idx==1
        [n, m] = deal(size(A,1), size(B,2));  % n=6, m=6
        % Cost matrices now sized for 6-state + 6-integrators
        Qx = eye(n);
        Qu = 0.1*eye(m);
        Qi = 10;
        Kx_all  = zeros(m, n,  Ngrid);
        Ki_all  = zeros(m, 6,  Ngrid);
      end

      % Store trim angles
      grid_pts(idx,:) = [phi0, theta0, psi0];

      % Design LQR + integrators for this (A,B)
      [Kx_all(:,:,idx), Ki_all(:,:,idx)] = ...
         designHoverControllerAtTrim(A, B, Qx, Qu, Qi);
    end
  end
end

% Save for Simulink
save('gainGrid3D.mat', 'phi_vals','theta_vals','psi_vals', ...
                      'grid_pts','Kx_all','Ki_all');
