function [sys, A, B, C, D] = rovHoverModel()
  % Load physical parameters
  p = rovParameters();      
  
  % Rigid-body + added mass
  Mrb = blkdiag( p.mass*eye(3), p.I );
  MA  = -diag([p.Ma_lin; p.Ma_rot]);
  MR  = Mrb + MA;
  
  % Linear damping at hover
  D0  = diag(p.D_lin);      

  % Build A, B, C, D
  A = [ zeros(6),       eye(6);
        zeros(6), -MR\D0 ];
  B = [ zeros(6);
        MR\eye(6) ];

  C = eye(12);
  D = zeros(12,6);

  sys = ss(A,B,C,D);
end
