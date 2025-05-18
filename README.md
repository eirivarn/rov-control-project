# Vehicle Kinematic Control Demo

## Files

- `simulationParameters.m`     : kinematic sim settings & body-rates  
- `computeRotationMatrices.m`  : build R(φ,θ,ψ) & T(φ,θ)  
- `computeKinematicsMap.m`     : η̇ = J(η)·ν  
- `simulateKinematics.m`       : runs kinematic sim with `ode45`  
- `visualizeTrajectory.m`      : live animation + GIF export (box or STL)  
- `controlParameters.m`        : controller gains, setpoints, timing  
- `vehicleControlModel.slx`    : Simulink plant + controller  
- `simulateControlSystem.m`    : runs the closed-loop sim & visualizes  

## Quickstart

1. Run the open-loop kinematic sim:
   ```matlab
   simulateKinematics
   visualizeTrajectory('box','open_loop_box.gif')
