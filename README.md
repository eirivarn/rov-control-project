# ROV Control Project - How to Run Simulations

This guide provides basic steps to run the simulations and scripts associated with the ROV Control Project. Ensure all project files (MATLAB scripts `.m`, Simulink models `.slx`, and `rov_model.stl`) are in your MATLAB path or current directory.

## Part 1: Pure Kinematic Simulation

This simulation demonstrates the ROV's motion based on kinematics only, without forces or dynamics.

1.  **Generate Test Trajectory:**
    * Open MATLAB.
    * Run the script:
        ```matlab
        makeTestTrajectory
        ```
    * This creates `testTrajectory_nu_split.mat` with timeseries data. 

2.  **Open and Configure Simulink Model:**
    * In MATLAB, type:
        ```matlab
        open('rov_kinematics_level0.slx')
        ```
    * In the Simulink model, go to "Model Configuration Parameters" (or press Ctrl+E).
    * In the "Solver" pane, set the "Stop time" to `t(end)` (where `t` is from `testTrajectory_nu_split.mat`). [cite: 39]

3.  **Run Kinematic Simulation:**
    * In the MATLAB command window, type:
        ```matlab
        sim('rov_kinematics_level0')
        ```
    * This will log `t_sim` and `eta_sim` to the MATLAB workspace. 

4.  **Animate Trajectory:**
    * After the simulation, run in MATLAB:
        ```matlab
        visualizeTrajectory('stl', 'kinematic_test_trajectory.gif', 5, t_sim, eta_sim, eta_sim);
        ```
    * This uses `rov_model.stl` to create `kinematic_test_trajectory.gif`. 
    * If you dont have downloaded the STL file from BlueROV2 Heavy, use "box" instead of "stl"

5.  **(Optional) Interactive Kinematic Testing:**
    * To manually test frame conventions:
        ```matlab
        interactiveROVControl()
        ```
    * This GUI allows you to manipulate the ROV's pose. 

## Part 2: Linearized Hover Model Verification

This script checks the linearized state-space model of the ROV around a hover equilibrium against the full nonlinear dynamics. 

1.  **Run Verification Script:**
    * In MATLAB, run:
        ```matlab
        test_hover_allDOF_verbose
        ```
    * The script automatically performs stability, controllability, observability, modal analysis, time-domain comparisons, and error-bound checks.

## Part 3: Controller and Observer - Setup and Linear Analysis

This involves setting up the main LTI plant, LQR controller gains, and observer gains using parameters tuned for better Simulink performance, and then performing a linear analysis on this configuration.

1.  **Configure and Run Main Setup Script:**
    * Open the `main.m` script.
    * Verify/adjust sensor toggles (e.g., `usePressure`, `useIMU`) at the top as needed. The LQR and observer parameters in this script are set to values found effective in Simulink.
    * Run the script in MATLAB:
        ```matlab
        main
        ```
    * This script defines the system matrices (`A_aug`, `B_aug`), LQR gains (`Kx`, `Ki`), observer gains (`L_sens`), and other parameters in the MATLAB workspace. It uses LQR parameters $Q_x = \mathrm{diag}([2.5\mathbf{I}_6, \mathbf{0}_6, \mathbf{0}_8])$, $Q_u = 0.05\mathbf{I}_8$, $Q_i = 0.001$, and observer parameters $Q_n = 0.01\mathbf{I}_{20}$ with sensor noise variances aligned with the report.

2.  **Run Controller and Observer Linear Analysis Script:**
    * Open the `test_controller_observer.m` 
    * This script should be configured to use the parameters set by `main.m` or have its tunable parameters (Section 0) set to the same values as in `main.m` (LQR: $Q_x = \mathrm{diag}([2.5\mathbf{I}_6, \mathbf{0}_6, \mathbf{0}_8])$, $Q_u \text{ scalar} = 0.05$, $Q_i = 25$; Observer: $Q_n \text{ scalar} = 0.01$, $R_n$ using report-aligned sensor variances).
        *(Note: The $Q_i=25$ in the test script's analysis parameters differs from the $Q_i=0.001$ in `main.m` for Simulink. 
    * Run the script in MATLAB:
        ```matlab
        test_controller_observer 
        ```
    * This performs checks like mixing matrix rank, Gramian analysis, closed-loop eigenvalues, step-response metrics, and observer performance for the *linear system configuration*.

## Part 4: Full Nonlinear Simulink Simulation

This involves running the complete ROV control system in the nonlinear Simulink environment. The parameters for the controller and observer are typically set by running `main.m` first (Step 3.1).

1.  **Ensure Parameters are Loaded:**
    * Run `main.m` (as in Step 3.1) to load all necessary variables (`A_aug`, `B_aug`, `Kx`, `Ki`, `L_sens`, `u0_cmd`, etc.) into the MATLAB workspace.

2.  **Open the Main Simulink Model:**
    * Identify the main Simulink file for the nonlinear plant with the controller and observer (e.g., `rov_nonlinear_control_system.slx` 
    * Open it in MATLAB:
        ```matlab
        open('rov_kinematics_level3.slx.slx') 
        ```

4.  **Run the Nonlinear Simulation:**
    * Click the "Run" button in the Simulink model window.
    * Observe the outputs (e.g., scopes, data logging) to evaluate performance. This is where you would verify the effectiveness of the parameters from `main.m` that were tuned for Simulink. 

-