function visualizeTrajectory(mode, outFile, delayTime, meshScale)
% VISUALIZETRAJECTORY  Animate & save a GIF of ROV pose (box or STL).
%   visualizeTrajectory(mode,outFile,delayTime,meshScale)
%     mode      – 'box' or 'stl'
%     outFile   – output .gif filename
%     delayTime – pause between frames (s)
%     meshScale – scale factor for the mesh (default 0.005)

  if nargin < 4, meshScale = 0.005; end
  if nargin < 3, delayTime = 0.05; end
  assert(ismember(mode,{'box','stl'}), 'mode must be ''box'' or ''stl''');

  % Load simulation data from base workspace
  t   = evalin('base','t_sim');     % may be longer than eta!
  eta = evalin('base','eta_sim');   % M×6 actual logged poses
  nSteps = size(eta,1);         % use the number of rows in eta

  % Create figure and axes
  hFig = figure('Color','w','Name','Animation','NumberTitle','off');
  ax   = axes('Parent',hFig);
  view(ax,3); rotate3d(ax,'on');
  grid(ax,'on'); axis(ax,'equal');
  xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
  hold(ax,'on');
  plot3(ax, eta(:,1), eta(:,2), eta(:,3), '-k', 'LineWidth',1.2);
  title(ax,'ROV Trajectory');

  % Prepare transform group
  tg = hgtransform('Parent',ax);

  % Base orientation fix (90° about Y, then 90° about Z)
  T0 = makehgtform('yrotate',pi/2,'zrotate',pi/2);

  switch mode
    case 'box'
      % Simple box geometry
      L = 1; W = 0.5; H = 0.3;
      V = [-L/2,-W/2,-H/2;  L/2,-W/2,-H/2;  L/2, W/2,-H/2; -L/2, W/2,-H/2; ...
           -L/2,-W/2, H/2;  L/2,-W/2, H/2;  L/2, W/2, H/2; -L/2, W/2, H/2];
      F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
      patch(ax, 'Vertices',V, 'Faces',F, ...
            'FaceColor',[0.8 0.6 0.4], ...
            'EdgeColor','k', ...
            'FaceLighting','gouraud', ...
            'Parent',tg);

    case 'stl'
      stlFile = fullfile(pwd,'rov_model.stl');
      assert(isfile(stlFile),'STL file not found: %s',stlFile);
      mesh = stlread(stlFile);
      if isa(mesh,'triangulation')
        F = mesh.ConnectivityList; V = mesh.Points;
      elseif isstruct(mesh) && isfield(mesh,'faces') && isfield(mesh,'vertices')
        F = mesh.faces; V = mesh.vertices;
      else
        error('Unsupported STL format.');
      end
      V = double(V);
      centroid = mean(V,1);
      V = (V - centroid) * meshScale;
      patch(ax, 'Faces',F, 'Vertices',V, ...
            'FaceColor',[0.8 0.8 1], ...
            'EdgeColor','none', ...
            'FaceLighting','gouraud', ...
            'Parent',tg);
      camlight(ax,'headlight');
      material(ax,'shiny');
  end

  drawnow;

  % Animate and write GIF
  for k = 1:nSteps
    % Extract pose (safe now)
    pos   = eta(k,1:3);
    phi   = eta(k,4);
    theta = eta(k,5);
    psi   = eta(k,6);

    % First place the vehicle at the pose, then apply model orientation
    Tpose = makehgtform( ...
      'translate',pos, ...
      'zrotate',psi, ...
      'yrotate',theta, ...
      'xrotate',phi );
    set(tg, 'Matrix', Tpose * T0);

    drawnow limitrate;
    pause(delayTime);

    % Capture frame and append to GIF
    try
      frame = getframe();
      [imind,cm] = rgb2ind(frame2im(frame),256);
      if k == 1
        imwrite(imind,cm,outFile,'gif','LoopCount',Inf,'DelayTime',delayTime);
      else
        imwrite(imind,cm,outFile,'gif','WriteMode','append','DelayTime',delayTime);
      end
    catch ME
      warning('Frame %d write failed: %s', k, ME.message);
    end
  end

  close(hFig);
  fprintf('Animation complete: %s (mode=%s)\n', outFile, mode);
end
