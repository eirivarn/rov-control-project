function [phi_vals, theta_vals, psi_vals, grid_pts, Kx_all, Ki_all] = buildGainGrid(Nphi, Ntheta, Npsi, filename)
% generateGainGrid3D  Build and save 3D grid of hover‐controller gains
%
%   [phi_vals,theta_vals,psi_vals,grid_pts,Kx_all,Ki_all] =
%       generateGainGrid3D(Nphi,Ntheta,Npsi)
%   computes LQR+integrator gains for an ROV trim grid of
%     phi ∈ linspace(-60°,60°,Nphi),
%     theta ∈ linspace(-60°,60°,Ntheta),
%     psi ∈ linspace(-180°,180°,Npsi),
%   and saves them to 'gainGrid3D.mat' in the current folder.
%
%   [...] = generateGainGrid3D(..., filename) uses the given filename
%   instead of 'gainGrid3D.mat'.
%
%   Outputs:
%     phi_vals,theta_vals,psi_vals  — vectors of length Nphi,Ntheta,Npsi
%     grid_pts                      — (Nphi*Ntheta*Npsi)×3 trim points
%     Kx_all                        — 6×6×(Nphi*Ntheta*Npsi) state‐feedback gains
%     Ki_all                        — 6×6×(Nphi*Ntheta*Npsi) integrator gains

  if nargin<4
    filename = 'gainGrid3D.mat';
  end

  %--- build trim‐angle vectors --------------
  deg = pi/180;
  phi_vals   = linspace(-60,  60,  Nphi) * deg;
  theta_vals = linspace(-60,  60,  Ntheta) * deg;
  psi_vals   = linspace(-180, 180, Npsi)   * deg;

  Ngrid   = Nphi * Ntheta * Npsi;
  grid_pts = zeros(Ngrid,3);

  % placeholders (will size on first iteration)
  Kx_all = [];
  Ki_all = [];

  idx = 0;
  for i = 1:Nphi
    for j = 1:Ntheta
      for k = 1:Npsi
        idx = idx + 1;
        phi0   = phi_vals(i);
        theta0 = theta_vals(j);
        psi0   = psi_vals(k);

        % linearize about this trim
        [A, B] = linearizeROV(phi0, theta0, psi0);

        if idx==1
          % on first pass, size everything and set cost weights
          [n, m] = deal(size(A,1), size(B,2));  % n=6, m=6
          Qx = eye(n);
          Qu = 0.1*eye(m);
          Qi = 10;
          Kx_all = zeros(m, n, Ngrid);
          Ki_all = zeros(m, 6, Ngrid);
        end
      end
    end
  end

  % save results
  save(filename, 'phi_vals','theta_vals','psi_vals', ...
                  'grid_pts','Kx_all','Ki_all');
end
