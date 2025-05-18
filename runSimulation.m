clear; clc; close all

% load parameters
params

% initial state
eta0 = zeros(6,1);

% integrate with ode45
[t, eta] = ode45(@(t,eta) kinematics(t,eta,nu), [0 Tfinal], eta0);

% store to workspace for easy plotting/animation
save('simulation.mat','t','eta');
disp('Simulation complete. Call animateTrajectory(t,eta).')
