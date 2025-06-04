function visualizeTrajectory(mode, outFile, fps, t_sim, eta_ref, eta_hat)
% VISUALIZETRAJECTORY  Animate & save GIF of ROV following two trajectories
%   visualizeTrajectory(mode,outFile,fps, ...
%                       t_sim,eta_real,eta_hat)
%
%   mode     – 'box' or 'stl'
%   outFile  – filename for the GIF
%   fps      – desired frames per second
%   t_sim    – N×1 time vector for actual trajectory
%   eta_real – N×6 reference poses [x y z phi theta psi]
%   eta_hat  – N×6 actual    poses [x y z phi theta psi]

    %% handle default fps
    if nargin < 3
        fps =5000;
    end

    %% create figure & axes
    hFig = figure('Color','w','Name','Animation','NumberTitle','off');
    ax   = axes('Parent',hFig);
    view(ax,3); rotate3d(ax,'on');
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
    title(ax,'ROV Trajectory: ref (red) vs actual (blue)');

    %% plot static traces
    plot3(ax, eta_ref(:,1), eta_ref(:,2), eta_ref(:,3), '--r', 'LineWidth',1.2);
    plot3(ax, eta_hat(:,1 ), eta_hat(:,2 ), eta_hat(:,3 ), '-b', 'LineWidth',1.2);

    %% build ROV mesh under hgtransform
    tg = hgtransform('Parent',ax);
    T0 = eye(4);
    switch mode
        case 'box'
            L=1; Wt=0.5; H=0.3;
            V = [-L/2,-Wt/2,-H/2;  L/2,-Wt/2,-H/2;  L/2,Wt/2,-H/2; -L/2,Wt/2,-H/2; ...
                 -L/2,-Wt/2, H/2;  L/2,-Wt/2, H/2;  L/2,Wt/2, H/2; -L/2,Wt/2, H/2];
            F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
            Rb = [0 0 1; 0 1 0; -1 0 0];
            V  = (Rb * V')';
            patch(ax,'Vertices',V,'Faces',F, ...
                  'FaceColor',[0.8 0.6 0.4], ...
                  'EdgeColor','k', ...
                  'Parent',tg, ...
                  'FaceLighting','gouraud');
            set(tg,'Matrix',T0);

        case 'stl'
            mesh = stlread('rov_model.stl');
            if isa(mesh,'triangulation')
                F = mesh.ConnectivityList; V = mesh.Points;
            else
                F = mesh.faces;          V = mesh.vertices;
            end
            if any(F(:)==0)
                F = F + 1;
            end
            V = double(V);
            centroid = mean(V,1);
            scale    = 0.005;
            V = (V - centroid) * scale;
            T0 = makehgtform('yrotate',pi/2,'zrotate',pi/2);
            patch(ax,'Faces',F,'Vertices',V, ...
                  'FaceColor',[0.8 0.8 1], ...
                  'EdgeColor','none', ...
                  'Parent',tg, ...
                  'FaceLighting','gouraud');
            set(tg,'Matrix',T0);
            camlight(ax,'headlight');
            material(ax,'shiny');

        otherwise
            error('Unknown mode ''%s''', mode);
    end  % switch

    %% compute uniform frame times
    t0      = t_sim(1);
    tEnd    = t_sim(end);
    dt      = 1 / fps;
    t_frames = t0 : dt : tEnd;          % row vector
    nFrames  = numel(t_frames);

    firstFrame = true;
    for i = 1 : nFrames
        tt = t_frames(i);

        % linear interpolation of position & Euler angles
        pos  = interp1(t_sim, eta_hat(:,1:3), tt, 'linear');
        angs = interp1(t_sim, eta_hat(:,4:6), tt, 'linear');
        phi   = angs(1); theta = angs(2); psi = angs(3);

        % update transform
        Tm = makehgtform('translate', pos, ...
                         'zrotate',  psi, ...
                         'yrotate',  theta, ...
                         'xrotate',  phi);
        set(tg, 'Matrix', Tm * T0);

        drawnow limitrate
        pause(0.01);
        % capture & write GIF frame
        frame = getframe(ax);
        [imind, cm] = rgb2ind(frame2im(frame), 256);
        if firstFrame
            imwrite(imind, cm, outFile, 'gif', ...
                    'LoopCount', Inf, 'DelayTime', dt);
            firstFrame = false;
        else
            imwrite(imind, cm, outFile, 'gif', ...
                    'WriteMode', 'append', 'DelayTime', dt);
        end
    end  % for

    close(hFig);
    fprintf('Done: %s (mode=%s), frames=%d at %.1f FPS\n', ...
            outFile, mode, nFrames, fps);

end  % function
