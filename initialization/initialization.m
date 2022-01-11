function [T, matchedPoints2, landmarks] = initialization(img1, img2, params,R0,t0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> the 2 images as GRAYSCALE and a struct of params
% output --> the transoformation from the 2 cameras frames
% Made by Roberto Pellerito as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% first we have to find the keypoints of the two image and match it
% we will do so by using harris (o shi-tommasi) scores 
% than we do non maximum suppression and select the highest scores
% than we find keypoints for each image, the return should be 
% 2 x num_keypoints containing 2d coordinates of each keypoint for
% the considered image

[keyP1, keyP2] = extractKeypoints(img1,img2,params);
[features1,features2,validpoints1,validpoints2] = extractDescriptors(img1, img2, keyP1, keyP2, params);
[p0,p1] = matchDescriptors(validpoints1,validpoints2,features1,features2, params);
[R,t,inlinerP1,inlinerP2] = findInitialPose(p0, p1, params);

[R_I_w,t_I_w] = cameraPoseToExtrinsics(R0,t0);
M0 = cameraMatrix(params.cam, R_I_w, t_I_w);

[R_c1_w,t_c1_w] = cameraPoseToExtrinsics(R0'*R,t+t0);
M1 = cameraMatrix(params.cam, R_c1_w, t_c1_w);

fprintf("\n Initialization transform \n");
%T = [R, t.'; 0, 0, 0, 1]
T = [R*R0, (t+t0).'; 0, 0, 0, 1]

[landmarks, reprojError, valididx] = triangulate(inlinerP1,inlinerP2,M0,M1);
matchedPoints2 = inlinerP2.Location(reprojError<=1 & valididx,:);
landmarks = landmarks(reprojError<=1 & valididx,:);

