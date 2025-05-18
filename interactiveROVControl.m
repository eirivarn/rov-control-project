function interactiveROVControl()
% INTERACTIVERO VCONTROL  Interactive GUI to control and visualize ROV pose
%   Launches a figure with sliders for X, Y, Z, Roll, Pitch, Yaw. Adjusting
%   sliders updates the 3D model (STL) in real time within a fixed frame.

% Load ROV mesh
mesh = stlread('rov_model.stl');
if isa(mesh,'triangulation')
    F = mesh.ConnectivityList;
    V = mesh.Points;
else
    if isfield(mesh,'faces') && isfield(mesh,'vertices')
        F = mesh.faces; V = mesh.vertices;
    elseif isfield(mesh,'ConnectivityList') && isfield(mesh,'Points')
        F = mesh.ConnectivityList; V = mesh.Points;
    else
        error('Unrecognized STL format');
    end
end

% Prepare vertices; ensure 1-based faces
F = double(F);
if any(F(:)==0), F = F + 1; end
V = double(V);

% Compute centroid and center the mesh for pivot about its center
centroid = mean(V,1);
V = V - centroid;

% Rotation options (choose one):
R_x90 = [1 0 0; 0 0 -1; 0 1 0];   % rotate 90° about X-axis
% R_y90 = [0 0 1; 0 1 0; -1 0 0];  % rotate 90° about Y-axis
% R_z90 = [0 -1 0; 1 0 0; 0 0 1];   % rotate 90° about Z-axis

% Apply initial rotation around mesh center
V = (R_x90 * V')';

% Scale the model
scaleFactor = 0.02;  % adjust smaller or larger as needed
V = V * scaleFactor;

% Create UI figure and fixed axes
fig = uifigure('Name','Interactive ROV Control','Position',[100 100 1000 600]);
ax  = uiaxes(fig,'Position',[25 150 600 425]);
view(ax,3); axis(ax,'equal');
lims = 5;
ax.XLim = [-lims lims]; ax.YLim = [-lims lims]; ax.ZLim = [-lims lims];
ax.XLimMode = 'manual'; ax.YLimMode = 'manual'; ax.ZLimMode = 'manual';
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
hold(ax,'on'); grid(ax,'on');

% Static reference axes
Lref = lims * 0.5;
quiver3(ax,0,0,0,Lref,0,0,'r','LineWidth',2,'MaxHeadSize',1);
quiver3(ax,0,0,0,0,Lref,0,'g','LineWidth',2,'MaxHeadSize',1);
quiver3(ax,0,0,0,0,0,Lref,'b','LineWidth',2,'MaxHeadSize',1);

% Patch ROV mesh under transform
tg = hgtransform('Parent',ax);
patch(ax,'Faces',F,'Vertices',V,'FaceColor',[0.8 0.8 1],...
      'EdgeColor','none','Parent',tg,'FaceLighting','gouraud','AmbientStrength',0.6);
camlight(ax,'headlight'); material(ax,'shiny');

% Initial pose
pose = struct('x',0,'y',0,'z',0,'roll',0,'pitch',0,'yaw',0);

% Slider panel and controls
labels = {'X','Y','Z','Roll','Pitch','Yaw'};
ranges = [-5 5; -5 5; -5 5; -pi pi; -pi/2 pi/2; -pi pi];
props  = fieldnames(pose);
panel = uipanel(fig,'Title','Controls','FontSize',14,...
                'Position',[650 25 350 550]);
for i = 1:6
    ypos = 450 - (i-1)*70;
    uilabel(panel,'Text',labels{i},'Position',[10 ypos+25 60 22]);
    sld = uislider(panel,'Position',[10 ypos 330 3],...
        'Limits',ranges(i,:), 'Value',pose.(props{i}));
    sld.ValueChangedFcn = @(src,event) sliderCallback(src,props{i});
end

% Draw initial pose
drawTransform();

% Callback: update pose and redraw
function sliderCallback(src,prop)
    pose.(prop) = src.Value;
    drawTransform();
end

% Apply transform to ROV mesh
function drawTransform()
    T = makehgtform('translate',[pose.x pose.y pose.z],...
                    'zrotate',pose.yaw,'yrotate',pose.pitch,'xrotate',pose.roll);
    set(tg,'Matrix',T);
end

end
