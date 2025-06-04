function [A_lin, B_lin] = linearizeROV(phi, theta, psi)
  % 1) get hover linearization at zero attitude
  [~, A0, B0] = rovWithActuators();  % A0:20x20, B0:20x8

  % 2) build R = Rz(psi)*Ry(theta)*Rx(phi)
  cph = cos(phi);   sph = sin(phi);
  cth = cos(theta); sth = sin(theta);
  cps = cos(psi);   sps = sin(psi);
  Rz = [ cps, -sps, 0; sps, cps, 0; 0,0,1 ];
  Ry = [ cth, 0, sth;   0,1,0;  -sth,0,cth ];
  Rx = [ 1,0,0; 0,cph,-sph; 0,sph,cph ];
  R_3 = Rz * Ry * Rx;  

  % 3) lift to 12×12 for the hydro states (6 pos+orient, 6 vel)
  T_hydro = blkdiag(R_3, R_3, R_3, R_3);  % 4 blocks of size 3
  %    block ordering must match your state ordering in rovWithActuators:
  %    [x;y;z], [phi;theta;psi], [u;v;w], [p;q;r]

  % 4) full 20×20 transform
  T = blkdiag(T_hydro, eye(8));

  % 5) similarity transform
  A_lin = T * A0 * T';
  B_lin = T * B0;
end
