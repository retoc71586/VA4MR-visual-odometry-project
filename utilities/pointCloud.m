function P = pointCloud(img1, p1, p2, K, T, show_figures)

% This function takes as imputs the points correspondeces and gives the
% coordinates of the 3-D points; furtermore it draws a 3-D point cloud with
% the points correspondeces

% Made as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich

%inputs p1 --> [3xN] (homogeneous) pixel coordinates of 1st image
%       p2 --> [3xN] (homogeneous) pixel coordinates of 2nd image
%       K  --> Intrinsic matrix
%       T  --> Transf. matrix from camera 1 to camera 2

%outputs: 
%       P --> [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation

R = T(1:3,1:3);
t = T(:,4);

P = linearTriangulation(p1, p2, K*eye(3,4),K*T);

%% Visualize the 3-D scene
if show_figures == 1;
subplot(1,3,1)
figure(1)

for i = 1:size(P,2)
    if P(3,i) < 0
       P(:,i) = zeros(1,4);
    end
end

plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3), zeros(3,1), 1.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -t;
plotCoordinateFrame(R',center_cam2_W, 1.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
grid

% Display matched points
subplot(1,3,2)
imshow(img1,[]);
hold on
plot(p1(1,:), p1(2,:), 'ys');
title('Image 1')

subplot(1,3,3)
plotCoordinateFrame(eye(3), zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
plotCoordinateFrame(R',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
%hold on
end

function  [hline,hhead] = plotCoordinateFrame( rotation, origin, len, colors)
% PLOTCOORDINATEFRAME - plots a 3d coordinate frame.
%
% [hline,hhead] = plotCoordinateFrame( rotation, origin, len)
%
%  In the resulting plot x is red, y is green and z is blue.
%
% origin   - the origin of the frame (rho_0_j0)
% rotation - A rotation f -> i that rotates points from the coordinate
%            frame to the inertial frame. (C_0j)
% colors   - the colors of the three axis arrows.
%            If omitted, the default is ['r'; 'g'; 'b']
% Returns the handles to the lines and the heads of the arrows.
if nargin < 3
    len = 1;
end

if nargin < 4
    colors = ['r';'g';'b'];
end

if size(rotation,1) ~= 3 || size(rotation,2) ~= 3
    error('rotation must be a 3x3 matrix');
end

if size(origin,1) ~= 3 || size(origin,2) ~= 1
    error('origin must be a 3x1 vector');
end
R = rotation';

[hline,hhead] = arrow3d(repmat(origin',3,1), repmat(origin',3,1) + len*R,15);

set(hline(1,1),'facecolor',colors(1,:));
set(hhead(1,1),'facecolor',colors(1,:));
set(hline(1,2),'facecolor',colors(2,:));
set(hhead(1,2),'facecolor',colors(2,:));
set(hline(1,3),'facecolor',colors(3,:));
set(hhead(1,3),'facecolor',colors(3,:));
