% generate_testTrajectory_nu_split.m
%---------------------------------------------------------------------------%
% Generates a time series nu(t) that stays within ±5° roll/pitch/yaw
%   Phase 1 (0–10 s): surge forward
%   Phase 2 (10–20 s): yaw turn +5° at constant rate
%   Phase 3 (20–30 s): sway right
%   Phase 4 (30–40 s): pitch up +5° at constant rate
%   Phase 5 (40–50 s): heave up
%   Phase 6 (50–60 s): roll +5° at constant rate
%   Phase 7 (60–70 s): surge forward
%   Phase 8 (70–80 s): all zero (stop)
%---------------------------------------------------------------------------%

dt   = 0.05;         % time step
Tseg = 10;           % seconds per phase
T    = 8*Tseg;       % total duration = 80 s
t    = (0:dt:T)';    % time vector (N×1)

% Pre-allocate nu = [u v w p q r]
nu = zeros(numel(t),6);

% Linear speeds (m/s)
U = 0.5;
V = 0;
W = 0;

% Angular rates to achieve ±5° in Tseg seconds
deg5   = 5 * pi/180;       % 5° in radians
R_yaw   = deg5 / Tseg;      % rad/s yaw (5° over 10s)
Q_pitch = deg5 / Tseg;      % rad/s pitch
P_roll  = deg5 / Tseg;      % rad/s roll

% Phase definitions: [t0, t1, channel, type, amplitude]
% type: 1 = constant-rate angular, 2 = half-sine bump on linear channel
phases = {
  0,     Tseg,   1, 2, U;         % surge bump (half-sine)
  Tseg,  2*Tseg, 6, 1, R_yaw;     % yaw +5° constant
  2*Tseg,3*Tseg, 2, 2, V;         % sway bump (zero amplitude)
  3*Tseg,4*Tseg, 5, 1, Q_pitch;   % pitch bump (half-sine) to 5°
  4*Tseg,5*Tseg, 3, 2, W;         % heave constant zero
  5*Tseg,6*Tseg, 4, 1, P_roll;    % roll bump (half-sine) to 5°
  6*Tseg,7*Tseg, 1, 1, U;         % surge bump
  7*Tseg,8*Tseg, 0, 2, 0;         % stop
};

% Populate nu based on phases
for i = 1:size(phases,1)
    t0  = phases{i,1};
    t1  = phases{i,2};
    ch  = phases{i,3};
    typ = phases{i,4};
    amp = phases{i,5};
    idx = t>=t0 & t< t1;
    if ch>0
        switch typ
            case 1  % constant-rate angular
                nu(idx,ch) = amp;
            case 2  % half-sine bump on linear channel
                ti = t(idx);
                nu(idx,ch) = amp * sin(pi*(ti - t0)/(t1 - t0));
        end
    end
end

% Wrap into timeseries for Simulink
ts_u = timeseries(nu(:,1), t, 'Name', 'u');
ts_v = timeseries(nu(:,2), t, 'Name', 'v');
ts_w = timeseries(nu(:,3), t, 'Name', 'w');
ts_p = timeseries(nu(:,4), t, 'Name', 'p');
ts_q = timeseries(nu(:,5), t, 'Name', 'q');
ts_r = timeseries(nu(:,6), t, 'Name', 'r');
save('testTrajectory_nu_zone.mat', 'ts_u', 'ts_v', 'ts_w', 'ts_p', 'ts_q', 'ts_r');

%{ Quick verification plot
labels = {'u','v','w','p','q','r'};
figure;
for ch = 1:6
    subplot(3,2,ch);
    plot(t, nu(:,ch), 'LineWidth',1.2);
    grid on;
    title(sprintf('Channel %s', labels{ch}));
    if ch>4, xlabel('Time (s)'); end
end
sgtitle('Test Trajectory: Motions within ±5° linear zone'); %}