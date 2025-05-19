%---------------------------------------------------------------------------%
% makeSquare6DOF.m  (Enhanced, corrected)
% Generates a time series nu(t) for testing all 6 DOF with smooth ramps
% and return trips, over eight 10-s phases (80 s total):
%   1) Surge forward
%   2) Sway  right
%   3) Heave up
%   4) Roll  positive
%   5) Pitch positive
%   6) Yaw   positive
%   7) Surge back
%   8) Sway  back
%---------------------------------------------------------------------------%

dt    = 0.05;          % finer time step
Tseg  = 10;            % seconds per phase
T     = 8*Tseg;        % total duration
t     = (0:dt:T)';     % column vector (M×1)

% Pre-allocate
nu = zeros(numel(t),6);  % [u v w p q r]

% Peak amplitudes
amps = [ 0.8, 0.6, 0.4,   ... % U, V, W (m/s)
         0.2, 0.15,0.12,  ... % P, Q, R (rad/s)
        -0.8, -0.6 ];        % return surge, sway (negative)

% Phase definitions: [start, end, channel, amplitude_index]
phases = [
   0,    Tseg,     1, 1;  % surge forward
   Tseg, 2*Tseg,   2, 2;  % sway right
   2*Tseg,3*Tseg,  3, 3;  % heave up
   3*Tseg,4*Tseg,  4, 4;  % roll
   4*Tseg,5*Tseg,  5, 5;  % pitch
   5*Tseg,6*Tseg,  6, 6;  % yaw
   6*Tseg,7*Tseg,  1, 7;  % surge back
   7*Tseg,8*Tseg,  2, 8;  % sway back
];

for i = 1:size(phases,1)
  t0 = phases(i,1);
  t1 = phases(i,2);
  ch = phases(i,3);
  ampIdx = phases(i,4);
  
  idx = t >= t0 & t < t1;
  % compute ramp only at these indices:
  ti = t(idx);
  ru = (sin(pi*(ti - t0)/(t1 - t0)) + 1)/2;  % 0→1→0 over phase
  
  nu(idx, ch) = amps(ampIdx) * ru;
end

% Wrap into a timeseries
ts_nu = timeseries(nu, t, 'Name', 'nu');
ts_nu.TimeInfo.Units = 'seconds';

Tend = t(end);   % where t is time vector

% Break out each DOF into its own timeseries
ts_u = timeseries(nu(:,1), t, 'Name', 'u');
ts_v = timeseries(nu(:,2), t, 'Name', 'v');
ts_w = timeseries(nu(:,3), t, 'Name', 'w');
ts_p = timeseries(nu(:,4), t, 'Name', 'p');
ts_q = timeseries(nu(:,5), t, 'Name', 'q');
ts_r = timeseries(nu(:,6), t, 'Name', 'r');

% Save them all to a MAT-file for easy loading in Simulink
save('square6DOF_nu_split.mat', ...
     'ts_u','ts_v','ts_w','ts_p','ts_q','ts_r');

% Quick plot
figure;
subplot(3,1,1)
plot(t, nu(:,1:3), 'LineWidth', 1.2);
grid on; legend('u','v','w'); ylabel('m/s');

subplot(3,1,2)
plot(t, nu(:,4:6), 'LineWidth', 1.2);
grid on; legend('p','q','r'); ylabel('rad/s');

subplot(3,1,3)
plot(t, nu, 'LineWidth', 1);
grid on; legend('u','v','w','p','q','r','Location','best');
ylabel('Command value'); xlabel('Time (s)');

if exist('sgtitle','file')
    sgtitle('6-DOF Square Test: Smooth Ramp & Return');
end
