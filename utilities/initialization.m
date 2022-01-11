function [T, keypoints_img1, keypoints_img2, landmarks] = initialization(img1, img2, K)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to bootstraps the initial camera poses.
% input --> the 2 images as GRAYSCALE
% output --> the transoformation from the 2 cameras frames
% Made by senecobis :rpellerito@ethz.ch as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% first we have to find the keypoints of the two image and match it
% we will do so by using harris (o shi-tommasi) scores 
% than we do non maximum suppression and select the highest scores
% than we find keypoints for each image, the return should be 
% 2 x num_keypoints containing 2d coordinates of each keypoint for
% the considered image

figures = false;

[p1,p2] = matchKeypoints(im2gray(img1),im2gray(img2));

[R,t, inliers] = findInitialPose(p1, p2, K);
T = [R,t];

keypoints_img1 = p1(inliers,:);
keypoints_img2 = p2(inliers,:);
num_keyp = size(keypoints_img1,1);
p1_ho = [keypoints_img1, ones(num_keyp,1)]';
p2_ho = [keypoints_img2, ones(num_keyp,1)]';

%Plot
if figures == false
    figure;
    showMatchedFeatures(img1,img2,p1(inliers,:),p2(inliers,:)); %Point correspondences
    landmarks = pointCloud(img1, img2, p1_ho, p2_ho, K, T); %3-D map
end
end
