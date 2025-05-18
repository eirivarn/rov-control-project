# Vehicle Kinematics Demo

This repository shows a minimalist 6-DOF kinematic model in MATLAB:
- `params.m`       : define body‐rate commands & sim settings  
- `runSimulation.m`: integrates using `ode45`  
- `kinematics.m`   : computes η̇ = J(η)·ν  
- `rotationMatrices.m`: helper to build R(φ,θ,ψ) & T(φ,θ)  
- `animateTrajectory.m`: plots & animates the result  

## Quick start  
1. Open MATLAB in this folder  
2. Run `runSimulation`  
3. Then call `animateTrajectory(t,η)`  
