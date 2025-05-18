function visualizeTrajectory(mode, outFile, delayTime)
% ANIMATETRAJECTORYWITHOPTIONS  Animate & save GIF with either a box or your STL model
%
%   animateTrajectoryWithOptions(mode, outFile)
%   animateTrajectoryWithOptions(mode, outFile, delayTime)
%
%   mode      = 'box' or 'stl'
%   outFile   = name of the GIF file to write, e.g. 'motion.gif'
%   delayTime = (optional) delay between GIF frames, default = 0.05

  if nargin<3, delayTime = 0.05; end
  assert(ismember(mode,{'box','stl'}), 'mode must be ''box'' or ''stl''');

  % load sim data
  S = load('simulation.mat','t','eta');
  t   = S.t;
  eta = S.eta;    % [x,y,z,φ,θ,ψ] rows

  % set up figure
  hFig = figure('Color','w','Name','Animation','NumberTitle','off');
  ax = axes('Parent',hFig);
  hold(ax,'on'), grid(ax,'on'), axis(ax,'equal')
  xlabel(ax,'X'), ylabel(ax,'Y'), zlabel(ax,'Z')
  title(ax,'Vehicle Trajectory')

  % plot static path
  plot3(ax, eta(:,1), eta(:,2), eta(:,3), '-k','LineWidth',1.2)

  % create transform node
  tg = hgtransform('Parent',ax);

  switch mode
    case 'box'
      % define a centered rectangular box (length L along local X, width W along Y, height H along Z)
      L = 1.0;  W = 0.5;  H = 0.3;  
      V = [ -L/2, -W/2, -H/2;
              L/2, -W/2, -H/2;
              L/2,  W/2, -H/2;
             -L/2,  W/2, -H/2;
             -L/2, -W/2,  H/2;
              L/2, -W/2,  H/2;
              L/2,  W/2,  H/2;
             -L/2,  W/2,  H/2 ];
      F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

      % rotate about local Y by +90° so top face becomes front face
      R_box = [ 0   0   1;
                0   1   0;
               -1   0   0 ];
      V = (R_box * V')';

      patch( 'Vertices', V, 'Faces', F, ...
             'FaceColor', [0.8 0.6 0.4], 'EdgeColor','k', ...
             'Parent', tg, 'FaceLighting','gouraud', 'AmbientStrength',0.6 );

    case 'stl'
      % load and preprocess your STL
      mesh = stlread('rov_model.stl');
      if isa(mesh,'triangulation')
        F = mesh.ConnectivityList;  V = mesh.Points;
      else
        % struct‐style fallback
        if isfield(mesh,'faces')
          F = mesh.faces; V = mesh.vertices;
        else
          F = mesh.ConnectivityList; V = mesh.Points;
        end
      end
      F = double(F); if any(F(:)==0), F = F+1; end
      V = double(V) * 0.1;  % adjust scale as needed

      patch( 'Faces', F, 'Vertices', V, ...
             'FaceColor',[0.8 0.8 1], 'EdgeColor','none', ...
             'Parent', tg, 'FaceLighting','gouraud','AmbientStrength',0.6 );
      camlight headlight
      material shiny
  end

  % animate & capture GIF
  for k = 1:numel(t)
    pos   = eta(k,1:3);
    phi   = eta(k,4);
    theta = eta(k,5);
    psi   = eta(k,6);

    % build transform: translate then Z,Y,X rotations
    T = makehgtform('translate',pos, ...
                    'zrotate',psi, ...
                    'yrotate',theta, ...
                    'xrotate',phi);
    set(tg,'Matrix',T);

    drawnow limitrate
    pause(0.005)  % adjust for on-screen speed

    % capture frame
    frame = getframe(hFig);
    [imind,cm] = rgb2ind(frame2im(frame),256);
    if k==1
      imwrite(imind,cm,outFile,'gif','LoopCount',Inf,'DelayTime',delayTime);
    else
      imwrite(imind,cm,outFile,'gif','WriteMode','append','DelayTime',delayTime);
    end
  end

  close(hFig)
  fprintf('Done! Mode="%s", GIF saved to "%s"\n', mode, outFile);
end
