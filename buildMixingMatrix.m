function Tmix = buildMixingMatrix()
  % Geometry (m)
  L     = 0.5750;         % total length
  W_h   = 0.2400;         % horizontal thruster span
  z_off = 0.100;          % vertical offset

  % Half-spans
  xf = +L/2; xa = -L/2;
  yh = +W_h/2; yha = -W_h/2;

  % 45° axes for X-config horizontals
  c = cos(pi/4);  s = sin(pi/4);
  e1 = [ +c; +s; 0 ];
  e2 = [ +c; -s; 0 ];
  e3 = [ -c; +s; 0 ];
  e4 = [ -c; -s; 0 ];

  % Positions
  r1 = [ xf;  yh;  0 ];
  r2 = [ xf; yha; 0 ];
  r3 = [ xa;  yh;  0 ];
  r4 = [ xa; yha; 0 ];

  % Vertical thrusters (heave only)
  e5 = [0;0;1];  r5 = [ xf;  yh; +z_off ];
  e6 = [0;0;1];  r6 = [ xf; yha; +z_off ];
  e7 = [0;0;1];  r7 = [ xa;  yh; -z_off ];
  e8 = [0;0;1];  r8 = [ xa; yha; -z_off ];

  % Build 6×8 matrix
  Tmix = [...
    [ e1; cross(r1,e1)], [ e2; cross(r2,e2)], ...
    [ e3; cross(r3,e3)], [ e4; cross(r4,e4)], ...
    [ e5; cross(r5,e5)], [ e6; cross(r6,e6)], ...
    [ e7; cross(r7,e7)], [ e8; cross(r8,e8)]  ...
  ];
end
