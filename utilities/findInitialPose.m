function [R,t, inlinerP1, inlinerP2] = findInitialPose(p1, p2, params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> p1 and p2 are two matrices N x 2 where N are the matched
% points, in first and second column we find the 2d coordinates of this points
% output --> the transoformation from the 2 cameras frames
% Made  as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% p1 e p2 hanno valori intermedi alle coordinate dei px e bisogna
% arrotondare

% Estimate the fundamental matrix
[F, inliers] = estimateFundamentalMatrix(p1, p2,'Confidence', 99.99,'Method','Ransac');

inlinerP1 = p1(inliers,:);
inlinerP2 = p2(inliers,:);

% calc the pose of the camera 1 as seen from the camera 0
[R,t,validPointsFraction] = relativeCameraPose(F, params.cam, inlinerP1, inlinerP2);

 if validPointsFraction < 0.5
    warning('[relativeCameraPose] ERROR: relative pose is invalid %f', validPointsFraction);
 end

end





