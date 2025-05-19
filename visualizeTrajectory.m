function visualizeTrajectory(mode, outFile, delayTime, t_ref, eta_ref, t_dyn, eta_dyn)
% VISUALIZETRAJECTORYREFDYN  Animate & save GIF of ROV following two trajectories
%   visualizeTrajectoryRefDyn(mode,outFile,delayTime,...
%       t_ref,eta_ref,t_dyn,eta_dyn)
%
%   mode     – 'box' or 'stl'
%   outFile  – filename for the GIF
%   delayTime– pause between frames (seconds)
%   t_ref    – N_ref×1 time vector for reference
%   eta_ref  – N_ref×6 reference poses [x y z phi theta psi]
%   t_dyn    – N_dyn×1 time vector for actual
%   eta_dyn  – N_dyn×6 actual poses

  if nargin<3, delayTime = 0.05; end

  %--- Create figure & axes ---
  hFig = figure('Color','w','Name','Animation','NumberTitle','off');
  ax   = axes('Parent',hFig);
  view(ax,3); rotate3d(ax,'on');
  hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
  xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
  title(ax,'ROV Trajectory: ref (red) vs actual (blue)');

  %--- Plot both full‐path traces ---
  % Reference in dashed red
  plot3(ax, eta_ref(:,1), eta_ref(:,2), eta_ref(:,3), '--r', 'LineWidth',1.2);
  % Actual in solid blue
  plot3(ax, eta_dyn(:,1), eta_dyn(:,2), eta_dyn(:,3), '-b', 'LineWidth',1.2);

  %--- Set up the ROV mesh transform ---
  tg = hgtransform('Parent',ax);
  T0 = eye(4);
  switch mode
    case 'box'
      L=1; Wt=0.5; H=0.3;
      V = [-L/2,-Wt/2,-H/2; L/2,-Wt/2,-H/2; L/2,Wt/2,-H/2; -L/2,Wt/2,-H/2; ...
           -L/2,-Wt/2,H/2;  L/2,-Wt/2,H/2;  L/2,Wt/2,H/2;  -L/2,Wt/2,H/2];
      F = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
      Rb=[0 0 1;0 1 0;-1 0 0]; V=(Rb*V')';
      patch(ax,'Vertices',V,'Faces',F,'FaceColor',[0.8 0.6 0.4],...
            'EdgeColor','k','Parent',tg,'FaceLighting','gouraud');
      set(tg,'Matrix',T0);
    case 'stl'
      mesh=stlread('rov_model.stl');
      if isa(mesh,'triangulation')
        F=mesh.ConnectivityList; V=mesh.Points;
      else
        F=mesh.faces; V=mesh.vertices;
      end
      if any(F(:)==0), F=F+1; end
      V=double(V); centroid=mean(V,1); scale=0.005;
      V=(V-centroid)*scale;
      T0=makehgtform('yrotate',pi/2,'zrotate',pi/2);
      patch(ax,'Faces',F,'Vertices',V,'FaceColor',[0.8 0.8 1],...
            'EdgeColor','none','Parent',tg,'FaceLighting','gouraud');
      set(tg,'Matrix',T0);
      camlight(ax,'headlight'); material(ax,'shiny');
    otherwise
      error('Unknown mode ''%s''',mode);
  end

  %--- Animate along the actual trajectory ---
  nSteps = size(eta_dyn,1);
  for k = 1:nSteps
    % get actual pose
    pos   = eta_dyn(k,1:3);
    phi   = eta_dyn(k,4);
    theta = eta_dyn(k,5);
    psi   = eta_dyn(k,6);

    % build transform from phi,theta,psi
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
  fprintf('Done: %s (mode=%s), ref steps=%d, actual steps=%d\n', ...
          outFile, mode, numel(t_ref), numel(t_dyn));
end
