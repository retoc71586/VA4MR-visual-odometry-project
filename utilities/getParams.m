function [params] = getParams(dataset)
%% General params
params.scores = 0.8;
params.angle_treshold = 3;

%% Initialization parameters
params.bootstrap_frames = [1, 3];   % for all datasets
params.uniq = false;
params.filt_size = 5;
params.max_ratio = 0.5;   

% keypoint detection and maching
params.method = 2; %0 = Harris, 1 = Surf; %2 Shi-Tomasi
params.feature_quality = 1e-4;
params.n_keypoints = 500;
params.descriptor_size = 11;        % has to be odd
assert(mod(params.descriptor_size, 2)==1)
params.matching_mode = 'patch_matching'; %'patch_matching' or 'klt'
params.max_num_keypoints = 100; %aumentare per più precisione -Lollo


%% keyframe parameters

    % KLT parameters
    params.lambda_key = 2;                  % maximum bidirectional error
    params.num_pyr_levels_key = 6;
    params.bl_size_key = [31, 31];
    params.max_its_key = 2000;

%% params for KITTI
if dataset == 0
    
    % Matching keypoints
    params.n_keypoints = 1200; % num of keypoints at each frame, it must be constant
    params.uniq = true;
    params.max_ratio = 0.97;
    params.feature_quality = 1e-4;
    params.filt_size = 3;
    
    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 6;
    params.bl_size = [31, 31];
    params.max_its = 2000;

    % P3P parameters
    params.max_num_trials = 100000;
    params.conf = 99;
    params.max_repr_err = 2;
    
%% params for MALAGA
elseif dataset == 1

    % Matching keypoints
    params.n_keypoints = 1000;
    params.uniq = true;
    params.max_ratio = 0.9;
    params.feature_quality = 1e-4;
    params.filt_size = 3;

    % KLT parameters
    params.lambda = 2;    % maximum bidirectional error
    params.num_pyr_levels = 5;
    params.bl_size = [31, 31];
    params.max_its = 40;

    % P3P parameters
    params.max_num_trials = 100000;
    params.conf = 99;
    params.max_repr_err = 1;
    
%% params for PARKING
elseif dataset == 2
    
    % Matching keypoints
    params.n_keypoints = 200;
    params.uniq = true;
    params.max_ratio = 0.9;

    % KLT parameters
    params.lambda = 2;                  % maximum bidirectional error
    params.num_pyr_levels = 6;
    params.bl_size = [31, 31];
    params.max_its = 200;

    % P3P parameters
    params.max_num_trials = 16000;
    params.conf = 99.973;
    params.max_repr_err = 0.6345;

end