function PrintPoses(T_W_c1, camera_name)
    

    figure(1)
    hold on
    
    %plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    %text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
    
    center_cam2_W = T_W_c1(1:3,end);
    plotCoordinateFrame(T_W_c1(1:3,1:3),T_W_c1(1:3,4), 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,camera_name,'fontsize',10,'color','k','FontWeight','bold');
    axis equal
    rotate3d on;
    grid
    title('Cameras relative poses')
    hold off
    pause(0.2)
end

