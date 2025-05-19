%---------------------------------------------------------------------------%
% Generates a time series nu(t) that:
%   Phase 1 (0–10 s): surge forward
%   Phase 2 (10–20 s): sway right
%   Phase 3 (20–30 s): heave up
%   Phase 4 (30–40 s): yaw turn +90°
%   Phase 5 (40–50 s): surge forward
%   Phase 6 (50–60 s): pitch turn +90°
%   Phase 7 (60–70 s): surge forward
%   Phase 8 (70–80 s): roll 360°
%   Phase 9 (80–90 s): all zero (stop)
%---------------------------------------------------------------------------%

dt    = 0.05;          % time step
Tseg  = 10;            % seconds per phase
T     = 9*Tseg;        % total duration = 90 s
t     = (0:dt:T)';     % time vector (N×1)

% Pre-allocate nu = [u v w p q r]
nu = zeros(numel(t),6);

% Define linear-speed amplitudes (m/s)
U = 0.5;
V = 0.4;
W = 0.3;

% Amplitudes for constant-rate turns
R_yaw   = (pi/2)/Tseg;   % 0.1571 rad/s constant
Q_pitch = (pi/2)/Tseg;   % 0.1571 rad/s
P_roll  = (2*pi)/Tseg;   % 0.6283 rad/s

% Phase definitions: [t0, t1, channel, type, amplitude]
% type = 1 for linear half-sine, 2 for constant
phases = {
  0,     Tseg,   1, 1, U;      % surge bump
  Tseg,  2*Tseg, 2, 1, V;      % sway bump
  2*Tseg,3*Tseg, 3, 1, W;      % heave bump
  3*Tseg,4*Tseg, 6, 2, R_yaw;  % yaw constant
  4*Tseg,5*Tseg, 1, 2, U;      % surge constant
  5*Tseg,6*Tseg, 5, 2, Q_pitch;% pitch constant
  6*Tseg,7*Tseg, 1, 2, U;      % surge constant
  7*Tseg,8*Tseg, 4, 2, P_roll; % roll constant
  8*Tseg,9*Tseg, 0, 2, 0       % stop
};

for i = 1:size(phases,1)
  t0   = phases{i,1};
  t1   = phases{i,2};
  ch   = phases{i,3};
  typ  = phases{i,4};
  amp  = phases{i,5};
  idx  = t>=t0 & t< t1;
  if ch>0
    switch typ
      case 1  % half-sine bump
        ti   = t(idx);
        nu(idx,ch) = amp * sin(pi*(ti - t0)/(t1 - t0));
      case 2  % constant
        nu(idx,ch) = amp;
    end
  end
end

% Wrap into timeseries for Simulink
ts_u = timeseries(nu(:,1), t,'Name','u');
ts_v = timeseries(nu(:,2), t,'Name','v');
ts_w = timeseries(nu(:,3), t,'Name','w');
ts_p = timeseries(nu(:,4), t,'Name','p');
ts_q = timeseries(nu(:,5), t,'Name','q');
ts_r = timeseries(nu(:,6), t,'Name','r');
save('custom6DOF_nu_split.mat','ts_u','ts_v','ts_w','ts_p','ts_q','ts_r');

% Quick verification plot
labels = {'u','v','w','p','q','r'};
figure;
for ch = 1:6
  subplot(3,2,ch);
  plot(t, nu(:,ch), 'LineWidth',1.2);
  grid on;
  title(sprintf('Channel %s',labels{ch}));
  if ch>4, xlabel('Time (s)'); end
end
sgtitle('Custom 6-DOF Test Trajectory');
