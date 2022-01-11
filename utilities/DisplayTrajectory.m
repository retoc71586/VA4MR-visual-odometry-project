function [S,hist_num_keyp_tot,hist_num_cand] = DisplayTrajectory(...
    T_w_c0, img, S, i, disp,hist_num_keyp_tot,hist_num_cand,enableScaling)
% function to plot trajectory

% Richiamo la funzione di plotting
fig = disp.fig;
border = disp.border; 
upper_border = disp.upper_border;
ax_img = disp.ax_img;
ax_path_landm = disp.ax_path_landm;
ax_num_keypoints = disp.ax_num_keypoints;
ax_pathfull = disp.ax_pathfull;

%% Init
n_keyp = 50;
cont = S.cont; %numero di nuovi keypoints
num_keyp = size(S.p,2); %num tot di keypoints
hist_num_keyp_tot(end+1) = num_keyp; %num of keypoints
hist_num_cand(end+1) = size(S.C,2); %num of candidates

T_I_C_new = T_w_c0(1:3,1:end); %Posizione camera

%aggiorno la history of positions
S.HoP(end+1,:) = T_I_C_new(1:3,4)'; %della Camera
S.HoL(:,end+1:end+size(S.X,2)) = S.X; %dei Landmarks

% Se ho troppi landm tolgo i più vecchi sennò diventa lento -L
if size(S.HoL,2) > 1000
    S.HoL = S.HoL(:,end-999:end);
end
newpos = S.HoP;

%% Immagine con i match
cla(ax_img);
hold(ax_img,'on')
imshow(img,[],'Parent', ax_img);


plot(S.C(1,:), S.C(2,:),...
    'Parent',ax_img,...
    'LineStyle','none',...
    'Marker','x',...
    'MarkerSize',3,...
    'MarkerEdgeColor','y'); % Candidate_keyp

plot(S.p(1,num_keyp-cont:num_keyp), S.p(2,num_keyp-cont:num_keyp),...
    'Parent',ax_img,...
    'LineStyle','none',...
    'Marker','.',...
    'MarkerSize',12,...
    'MarkerEdgeColor','b'); % New_keypoints 

plot(S.p(1,1:num_keyp-cont), S.p(2,1:num_keyp-cont),...
    'Parent',ax_img,...
    'LineStyle','none',...
    'Marker','.',...
    'MarkerSize',12,...
    'MarkerEdgeColor','r'); % Keypoints_utilized 

title(ax_img,['Frame ' int2str(i) ' Being Processed']);
legend(ax_img,'Candidate Keypoints', 'Newly added Keypoints',...
    'Keypoints being used', 'Location','northwest','Box','on');

hold(ax_img,'off')


%% Graph of the movement from the above
cla(ax_path_landm);

% Current Landm
plot(S.X(1,:), S.X(3,:),'Marker','.','Parent',ax_path_landm,'Color','b','LineStyle','none','MarkerSize',5);
hold(ax_path_landm,'on')

% All landmarks
plot(S.HoL(1,:), S.HoL(3,:),'Marker','.','Parent',ax_path_landm,'Color','g','LineStyle','none','MarkerSize',1);




% Trajectory
plot(newpos(end+1-min(size(newpos,1),n_keyp):end,1), ...
    newpos(end+1-min(size(newpos,1),n_keyp):end,3), ...
    'Parent', ax_path_landm, 'LineWidth', 3,'Color','r'); 


title(ax_path_landm,'Cameras trajectory in last 50 frames and Landmarks (upside view)')
legend(ax_path_landm, 'Landmarks being used','Old Landmarks',...
    'Trajectory of last 50 frames', 'Location','northwest','Box','on');

if enableScaling == false

    ylim(ax_path_landm,[newpos(end,3)-80, newpos(end,3)+80]);
    axis(ax_path_landm, 'equal')
    xlim(ax_path_landm,[newpos(end,1)-80, newpos(end,1)+80]);


else
    axis(ax_path_landm, 'equal')
    ylim(ax_path_landm,[newpos(end,3)-60, newpos(end,3)+60]);
    
    xlim(ax_path_landm,[newpos(end,1)-40, newpos(end,1)+40]);
    

end


grid(ax_path_landm, 'on')

hold(ax_path_landm,'off')


%% Grafico Numero di Keypoints
hold(ax_num_keypoints,'on')
x = [1:size(hist_num_keyp_tot,2)];
plot(x,hist_num_keyp_tot,'Parent',ax_num_keypoints,...
    'Color','r', 'LineWidth', 1);
plot(x,hist_num_cand,'Parent',ax_num_keypoints,...
    'Color','b', 'LineWidth', 1);
xlim(ax_num_keypoints,[max(x(end)-75,1), x(end)]);
title(ax_num_keypoints,'Num Keypoints in last 75 frames')
legend(ax_num_keypoints, 'Keypoints being used',...
    'Candidate keypoints (no Landmark yet)',...
    'Location','northeast','Box','on');
grid(ax_num_keypoints,'on')

hold(ax_num_keypoints,'off')

%% Grafico FullTrajectory
cla(ax_pathfull)
hold(ax_pathfull,'on')
plot(newpos(:,1),newpos(:,3),'Parent', ax_pathfull, 'LineWidth', 3,...
    'Color','g'); 
title(ax_pathfull,'Full Trajectory')
axis(ax_pathfull, 'equal')

%grid(ax_pathfull, 'on')
hold(ax_pathfull,'off')

end


%% 3d Points
% subplot(2,2,2)
% plot3(S.X(1,:),S.X(2,:),S.X(3,:),'o'), hold on
% if 1 % Se si vuole pure vedere la traiettoria in 3D
%     plot3(newpos(:,1), newpos(:,2), newpos(:,3), 'LineWidth', 3);
% end
% if 1 %Se si vuole anche plottare la posizione della camera
%     camera_name = append('camera', string(i));
%     center_cam2_W = T_w_c0(1:3,end);
%     plotCoordinateFrame(T_w_c0(1:3,1:3)',T_w_c0(1:3,4), 5);
%     text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,camera_name,'fontsize',10,'color','k','FontWeight','bold');
%     axis equal
%     rotate3d on;
%     grid
%     title('Cameras relative poses and 3d landmarks')
% end
% 
% hold off
% 
% 
% 