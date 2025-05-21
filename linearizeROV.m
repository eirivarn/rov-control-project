function [A_lin, B_lin] = linearizeROV(phi, theta)
% LINEARIZEROV  Return small-perturbation A,B about a trim at (phi,theta,0)
%
%   [A_lin,B_lin] = linearizeROV(phi,theta) uses your existing
%   rovParameters() and rovHoverModel() to get the nominal
%   hover-point linearization (at phi=theta=0), then transforms
%   that A,B into the body frame rotated by phi about X and theta about Y.

  %--- 1) Get nominal hover A0,B0 ---
  sys0 = rovHoverModel();
  [A0, B0] = ssdata(sys0);  % A0 is 12×12, B0 is 12×6

  %--- 2) Build the rotation matrix R(φ,θ) for the body axes ---
  % Note: this is a 3×3 rotation of the body frame.
  R_phi   = [1      0           0;
             0 cos(phi) -sin(phi);
             0 sin(phi)  cos(phi)];
  R_theta = [ cos(theta) 0 sin(theta);
                    0    1     0;
             -sin(theta) 0 cos(theta)];
  R12 = R_phi * R_theta;     % first pitch, then roll

  %--- 3) Lift to 12×12 block-diag for position+velocity subspace ---
  T = blkdiag(R12, R12, R12, R12);  % Now T is 12x12

  %--- 4) Rotate A0,B0 into this trimmed frame ---
  A_lin = T * A0 * T';       % similarity transform
  B_lin = T * B0;            % inputs rotate with body

end
