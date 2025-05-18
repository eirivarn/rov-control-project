function visualizeTrajectory(mode, outFile, delayTime)
% VISUALIZETRAJECTORY  Animate & save GIF of ROV pose
%   Assumes `t` (Nx1) and `eta` (Nx6) are already in workspace.

  if nargin<3, delayTime = 0.05; end
  assert(ismember(mode,{'box','stl'}), 'mode must be ''box'' or ''stl''');

  %% 1) Grab t & eta
  % If you saved via To Workspace as arrays:
  %   t   = evalin('base','t');
  %   eta = evalin('base','eta');
  t   = evalin('base','t');
  eta = evalin('base','eta');
  nSteps = size(eta,1);

  %% 2) Prepare figure
  hFig = figure('Color','w','Name','Animation','NumberTitle','off');
  ax   = axes('Parent',hFig);
  view(ax,3); rotate3d(ax,'on');
  hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
  xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
  title(ax,'ROV Trajectory');
  plot3(ax,eta(:,1),eta(:,2),eta(:,3),'-k','LineWidth',1.2);

  %% 3) Create transform group
  tg = hgtransform('Parent',ax);
  T0 = eye(4);

  %% 4) Draw geometry
  switch mode
    case 'box'
      L=1; W=0.5; H=0.3;
      V = [-L/2,-W/2,-H/2; L/2,-W/2,-H/2; L/2,W/2,-H/2; -L/2,W/2,-H/2; ...
           -L/2,-W/2,H/2;  L/2,-W/2,H/2;  L/2,W/2,H/2;  -L/2,W/2,H/2];
      F = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
      Rb = [0 0 1;0 1 0;-1 0 0]; V=(Rb*V')';
      patch(ax,'Vertices',V,'Faces',F,'FaceColor',[0.8 0.6 0.4],...
            'EdgeColor','k','Parent',tg,'FaceLighting','gouraud');
      set(tg,'Matrix',T0);

    case 'stl'
      mesh = stlread('rov_model.stl');
      if isa(mesh,'triangulation')
        F=mesh.ConnectivityList; V=mesh.Points;
      elseif isstruct(mesh)
        F=mesh.faces; V=mesh.vertices;
      else
        error('Cannot read STL');
      end
      if any(F(:)==0), F=F+1; end
      V = double(V);
      centroid = mean(V,1);        % compute mesh center
      scaleFactor = 0.005;          % adjust to your desired size
      V = (V - centroid) * scaleFactor;
      T0 = makehgtform('yrotate',pi/2,'zrotate',pi/2);
      patch(ax,'Faces',F,'Vertices',V,'FaceColor',[0.8 0.8 1],...
            'EdgeColor','none','Parent',tg,'FaceLighting','gouraud');
      set(tg,'Matrix',T0);
      camlight(ax,'headlight'); material(ax,'shiny');
  end

  %% 5) Animate & write GIF
  for k = 1:nSteps
    pos   = eta(k,1:3);
    phi   = eta(k,4);
    theta = eta(k,5);
    psi   = eta(k,6);

    Tm = makehgtform('translate',pos, ...
                     'zrotate',psi, ...
                     'yrotate',theta, ...
                     'xrotate',phi);
    set(tg,'Matrix', Tm * T0);

    drawnow limitrate;
    pause(delayTime);

    frame = getframe(ax);
    [imind,cm] = rgb2ind(frame2im(frame),256);
    if k==1
      imwrite(imind,cm,outFile,'gif','LoopCount',Inf,'DelayTime',delayTime);
    else
      imwrite(imind,cm,outFile,'gif','WriteMode','append','DelayTime',delayTime);
    end
  end

  close(hFig);
  fprintf('Animation complete: %s (%s mode)\n', outFile, mode);
end
