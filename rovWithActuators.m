function [sys_aug, A_aug, B_aug, C_aug, D_aug, Tmix] = rovWithActuators()
  % 1) load 12-state hydro model
  [~, A, B, C, D] = rovHoverModel();

  % 2) actuator params
  K     = 40;      % N per unit command
  tau   = 0.1;     % time constant [s]

  % 3) mixing
  Tmix = buildMixingMatrix();  % 6×8
  n_h = size(A,1);                     % =12
  n_a = 8;

  % 4) form A_aug, B_aug
  A_aug = [ ...
    A,               B * Tmix * K; 
    zeros(n_a,n_h),  - (1/tau)*eye(n_a) ];

  B_aug = [ ...
    zeros(n_h,n_a);
    (1/tau)*eye(n_a) ];

  % 5) outputs: hydro states only
  C_aug = [ eye(n_h), zeros(n_h,n_a) ];
  D_aug = zeros(n_h,n_a);

  % 6) wrap as LTI
  sys_aug = ss(A_aug, B_aug, C_aug, D_aug);
end
