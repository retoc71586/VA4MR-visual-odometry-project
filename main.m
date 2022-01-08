%% Init
close all
clear all
clc

%% Import
addpath('utilities/'); addpath('continuos/'); addpath('initialization/'); %addpath('test_continuos\')

%% Setup
ds = 0; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'kitti';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truths = load([kitti_path '/poses/05.txt']);
    pose = ground_truths(3,:);
    ground_truth_pose = reshape(pose, [4,3]);
    ground_truth_pose = ground_truth_pose';
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = 'malaga-urban-dataset-extract-07';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = 'parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);     
    ground_truths = load([parking_path '/poses.txt']);
    pose = ground_truths(3,:);
    ground_truth_pose = reshape(pose, [4,3]);
    ground_truth_pose = ground_truth_pose';
else
    assert(false);
end

%% Load global parameters

params = getParams(ds);
params.cam = cameraParameters('IntrinsicMatrix', K.');

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1 4];
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%%%%%%%%%%%%%%%%%% testing on main 

[T_w_c, keypoints_img1, landmarks] = initialization(img0, img1, params,eye(3),[0,0,0]);

S.p = keypoints_img1.';
fprintf('number of keypoints:%d  \n',size(S.p,2));
S.X = landmarks.';
S.C = S.p;
S.F = S.p;
S.T = reshape(T_w_c,[16,1]).*ones(16,height(keypoints_img1));
       
%fprintf("ground truth")
prev_img = img1;

% Init plotting
T_w_c0 = [T_w_c; 0 0 0 1];
PrintPoses(eye(4),'world frame')
PrintPoses(T_w_c0,'first camera')

% History of camera positions
S.HoP = zeros(1,3);

% History of landmarks
S.HoL = zeros(3,1);
%% Continuous operation

range = (bootstrap_frames(2)+1):1:last_frame;

disp = init_disp_vo(img1);
hist_num_keyp_tot = 0;
hist_num_cand = 0;

for i = range
    %fprintf('numero keypoints:%d  \n',length(S.p));
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
% here put functions to plot results : trajectorie, keypoints  and landmarks
% firstly process frame needs an initialization of S0, according to the
% dimension requested. This init can be done through initialization (by changing it)

if max(size(S.p)) > 20

    [S, T_w_c1] = processFrame(S, prev_img, image, K, params);
    T_w_c0 = T_w_c1;

else

    fprintf("reinitialization");
    [T_w_c, keypoints_img1, landmarks] = ...
        initialization(prev_img, image, params, eye(3),[0 0 0]);
    S.p = keypoints_img1.';
    S.X = landmarks.';
    T_w_c0 = T_w_c0*T_w_c;
    T_w_c0(1:3,1:3) = T_w_c0(1:3,1:3).';

    for point = 1:size(S.X,2)
        real_point = T_w_c0*[S.X(:,point);1];
        S.X(:,point) = real_point(1:3);
    end

    S.C = S.p;
    S.F = S.p;
    T_w_c0(1:3,1:3) = T_w_c0(1:3,1:3).';
    S.T = reshape(T_w_c0,[16,1]).*ones(16,height(keypoints_img1));
    
end


[S,hist_num_keyp_tot, hist_num_cand] = DisplayTrajectory(T_w_c0, image, S, ...
    i, disp,hist_num_keyp_tot, hist_num_cand);
prev_img = image;
% Makes sure that plots refresh.    
pause(0.2);



end







