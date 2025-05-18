function animateTrajectoryGIF(outFile, delayTime)
% ANIMATEANDSAVEGIF  Show the vehicle animation and save it as a GIF
%
%   animateAndSaveGif(outFile)
%   animateAndSaveGif(outFile, delayTime)
%
% - outFile   : name of the .gif to create (e.g. 'motion.gif')
% - delayTime : (optional) seconds between GIF frames (default = 0.05)
%
% Requires:
%   - simulation.mat (with variables t, eta)
%   - rotationMatrices.m in the same folder

  if nargin<2
    delayTime = 0.05;    % default ~20 fps
  end

  % load simulation data
  data = load('simulation.mat','t','eta');
  t   = data.t;
  eta = data.eta;

  % unpack trajectory
  x = eta(:,1); y = eta(:,2); z = eta(:,3);

  % create visible figure
  hFig = figure('Color','w','Name','Vehicle Animation','NumberTitle','off');
  plot3(x,y,z,'-k','LineWidth',1.5); hold on, grid on, axis equal
  xlim([min(x) max(x)]); ylim([min(y) max(y)]); zlim([min(z) max(z)])
  xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
  title('Vehicle Trajectory & Body Frame')

  % initial body axes
  L = 0.5;    % arrow length
  [R0,~] = rotationMatrices(eta(1,4),eta(1,5),eta(1,6));
  O0 = eta(1,1:3)';
  qx = R0*[1;0;0]*L;
  qy = R0*[0;1;0]*L;
  qz = R0*[0;0;1]*L;
  h1 = quiver3(O0(1),O0(2),O0(3), qx(1),qx(2),qx(3), 'r','LineWidth',2,'MaxHeadSize',1);
  h2 = quiver3(O0(1),O0(2),O0(3), qy(1),qy(2),qy(3), 'g','LineWidth',2,'MaxHeadSize',1);
  h3 = quiver3(O0(1),O0(2),O0(3), qz(1),qz(2),qz(3), 'b','LineWidth',2,'MaxHeadSize',1);

  % loop through each time step
  for k = 1:length(t)
    % new pose & orientation
    O  = eta(k,1:3)';
    [R,~] = rotationMatrices(eta(k,4),eta(k,5),eta(k,6));
    qx = R*[1;0;0]*L;
    qy = R*[0;1;0]*L;
    qz = R*[0;0;1]*L;

    % update quivers
    set(h1, 'XData',O(1),'YData',O(2),'ZData',O(3), ...
            'UData',qx(1),'VData',qx(2),'WData',qx(3));
    set(h2, 'XData',O(1),'YData',O(2),'ZData',O(3), ...
            'UData',qy(1),'VData',qy(2),'WData',qy(3));
    set(h3, 'XData',O(1),'YData',O(2),'ZData',O(3), ...
            'UData',qz(1),'VData',qz(2),'WData',qz(3));

    drawnow limitrate   % update the figure
    pause(0.005)        % tiny pause so you see the smooth motion

    % capture frame for GIF
    frame = getframe(hFig);
    [imind, cm] = rgb2ind(frame2im(frame), 256);
    if k==1
      imwrite(imind, cm, outFile, 'gif', 'LoopCount', Inf, 'DelayTime', delayTime);
    else
      imwrite(imind, cm, outFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
  end

  fprintf('Animation complete. GIF saved as "%s"\n', outFile);
end
