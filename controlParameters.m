%% controlParameters.m

%–– Simulation timing ––
Ts     = 0.01;   % [s] sample time for discrete blocks
Tfinal = 20;     % [s] total sim time

%–– Desired inertial pose ––
%   [x; y; z; roll; pitch; yaw]
eta_des = [ 5; 0; -2; 0; 0; pi/4 ];

%–– Position loop gains (P‐only) ––
Kp_pos = diag([0.5, 0.5, 0.5]);  % proportional on x,y,z
Ki_pos = zeros(3);               % integral if you want

%–– Attitude loop gains (P‐only) ––
Kp_att = diag([1.0, 1.0, 1.0]);  % proportional on roll,pitch,yaw
Ki_att = zeros(3);

%–– Body‐rate limits (for saturation) ––
maxRates = [0.5; 0.5; 0.5];  % [p;q;r] max angular speeds
