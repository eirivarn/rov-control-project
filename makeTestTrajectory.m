% makeTestTrajectoryFull.m
% Generates time series ν(t) = [u v w p q r] and η(t) = [x y z φ θ ψ] over 90 s:
%   Phase 1 ( 0–10 s): surge forward
%   Phase 2 (10–20 s): sway right
%   Phase 3 (20–30 s): heave up
%   Phase 4 (30–40 s): yaw turn +90°
%   Phase 5 (40–50 s): surge forward
%   Phase 6 (50–60 s): pitch turn +90°
%   Phase 7 (60–70 s): surge forward
%   Phase 8 (70–80 s): roll 360°
%   Phase 9 (80–90 s): all zero (stop)

clear; clc;

% time parameters
dt    = 0.05;          % time step [s]
Tseg  = 10;            % seconds per phase
T     = 9 * Tseg;      % total duration [s]
t     = (0:dt:T)';     % time vector (N×1)
N     = numel(t);

% Pre-allocate ν = [u v w p q r]
nu = zeros(N,6);

% Linear-speed amplitudes [m/s]
U = 0.5;
V = 0.4;
W = 0.3;

% Angular-rate amplitudes [rad/s]
R_yaw   = (pi/2) / Tseg;   % yaw +90° in Tseg
Q_pitch = (pi/2) / Tseg;   % pitch +90° in Tseg
P_roll  = (2*pi) / Tseg;   % roll 360° in Tseg

% Phase definitions: [t0, t1, channel, type, amplitude]
% channel: 1=u,2=v,3=w,4=p,5=q,6=r; type: 1=half-sine, 2=constant
phases = {
    0,     Tseg,   1, 1, U;      % surge bump
    Tseg,  2*Tseg, 2, 1, V;      % sway bump
    2*Tseg,3*Tseg, 3, 1, W;      % heave bump
    3*Tseg,4*Tseg, 6, 2, R_yaw;  % yaw turn
    4*Tseg,5*Tseg, 1, 2, U;      % surge constant
    5*Tseg,6*Tseg, 5, 2, Q_pitch;% pitch turn
    6*Tseg,7*Tseg, 1, 2, U;      % surge constant
    7*Tseg,8*Tseg, 4, 2, P_roll; % roll turn
    8*Tseg,9*Tseg, 0, 2, 0       % stop
};

% Fill ν according to phases
for i = 1:size(phases,1)
    t0  = phases{i,1};
    t1  = phases{i,2};
    ch  = phases{i,3};
    typ = phases{i,4};
    amp = phases{i,5};
    idx = t >= t0 & t < t1;
    if ch > 0
        switch typ
            case 1  % half-sine bump
                ti = t(idx);
                nu(idx,ch) = amp * sin(pi * (ti - t0) / (t1 - t0));
            case 2  % constant
                nu(idx,ch) = amp;
        end
    end
end

% Integrate η from ν
eta = zeros(N,6);                % η = [x y z φ θ ψ]
eta(1,:) = [0, 0, 0, 0, 0, 0];    % initial at origin, level

for k = 1:N-1
    % current Euler angles
    phi   = eta(k,4);
    theta = eta(k,5);
    psi   = eta(k,6);
    % build J1 (translation) and J2 (rotation) blocks
    J1 = [...
        cos(psi)*cos(theta), ...
       -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), ...
        sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
        sin(psi)*cos(theta), ...
        cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), ...
       -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
       -sin(theta), ...
        cos(theta)*sin(phi), ...
        cos(theta)*cos(phi)];
    J2 = [...
        1, sin(phi)*tan(theta), cos(phi)*tan(theta);
        0, cos(phi),           -sin(phi);
        0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    J = [J1, zeros(3); zeros(3), J2];

    % body velocities
    nu_k = nu(k,:)';

    % η̇ = J * ν
    eta_dot = J * nu_k;

    % Euler forward integration
    eta(k+1,:) = eta(k,:) + (dt * eta_dot)';
end

% Pack into timeseries objects
ts_u   = timeseries(nu(:,1), t, 'Name', 'u');
ts_v   = timeseries(nu(:,2), t, 'Name', 'v');
ts_w   = timeseries(nu(:,3), t, 'Name', 'w');
ts_p   = timeseries(nu(:,4), t, 'Name', 'p');
ts_q   = timeseries(nu(:,5), t, 'Name', 'q');
ts_r   = timeseries(nu(:,6), t, 'Name', 'r');
ts_eta = timeseries(eta,   t, 'Name', 'eta');

% Set time units
ts_u.TimeInfo.Units   = 'seconds';
ts_v.TimeInfo.Units   = 'seconds';
ts_w.TimeInfo.Units   = 'seconds';
ts_p.TimeInfo.Units   = 'seconds';
ts_q.TimeInfo.Units   = 'seconds';
ts_r.TimeInfo.Units   = 'seconds';
ts_eta.TimeInfo.Units = 'seconds';

% Save to MAT-file
save('testTrajectory_nu_full.mat', ...
     'ts_u','ts_v','ts_w','ts_p','ts_q','ts_r','ts_eta');
