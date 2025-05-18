%% simulateControlSystem.m

clear; clc; close all

% 1) grab your controller & sim settings
controlParameters

% 2) run the Simulink model
simOut = sim('vehicleControlModel', ...
             'StopTime', num2str(Tfinal), ...
             'FixedStep', num2str(Ts));

% 3) extract the logged signals
t   = simOut.tout;  
eta = simOut.logsout.getElement('eta').Values.Data;  % [Nx6]
nu  = simOut.logsout.getElement('nu').Values.Data;   % [Nx6]

% 4) visualize both open- and closed-loop
visualizeTrajectory('box','ctrl_box.gif',0.02);
visualizeTrajectory('stl','ctrl_stl.gif',0.04);
