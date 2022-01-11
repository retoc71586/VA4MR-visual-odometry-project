function [p1,p2] = matchKeypoints(I1,I2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to extract and match keypoints between two images.
% input --> the 2 images as GRAYSCALE
% output --> the matched locations in the 2 cameras frames
% Made as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     close all
%     I1 = im2gray(imread('parking/images/img_00000.png'));
%     I2 = im2gray(imread('parking/images/img_00002.png'));
figures = false;
method = 2; %0 = Harris, 1 = Surf;

%% corner detection
if method == 0
    points1 = detectHarrisFeatures(I1,'FilterSize',5,'MinQuality', 0.01); % points1 is a cornerPoints object
    points2 = detectHarrisFeatures(I2,'FilterSize',5,'MinQuality', 0.01);
elseif method == 1
    points1 = detectSURFFeatures(I1,'MetricThreshold',200);
    points2 = detectSURFFeatures(I2,'MetricThreshold',200);
elseif method == 2
    points1 = detectMinEigenFeatures(I1,'FilterSize',5,'MinQuality', 0.1); % points1 is a cornerPoints object
    points2 = detectMinEigenFeatures(I2,'FilterSize',5,'MinQuality', 0.1);
end

strongest1 = selectStrongest(points1,500); % selectStrongest is a method of cornerPoints
strongest2 = selectStrongest(points2,500);
if figures == true
    figure
    imshow(I1)
    hold on
    plot(strongest1)
    hold off
end


if figures == true
    figure
    imshow(I2)
    hold on
    plot(strongest2)
    hold off
end
%% feature extraction
    % After detecting the corners we need to extract the descriptors in order to
    % do the matching, extractFeatures does that.
    % The type of descriptor returned depends on the type of points1. In this
    % case it is a cornerPoints so we get FREAK descriptors, fast to compute and
    % both rotation and scale invariant.

    % we could set upright to true since the images are not rotated in parking
    % but this way we can use it for all the datasets; NB: rotation in radiants

    [features1,valid_points1] = extractFeatures(I1,strongest1,'BlockSize',11);
%     valid_points1
    [features2,valid_points2] = extractFeatures(I2,strongest2,'BlockSize',11);
%     valid_points2
   
    %esclude i keypoints che stanno sul bordo quindi ne perdo un po
    % size(points1)
    % size(valid_points1)

    % valid points is a cornerPoints object containing for each feature the px
    % location, cornerness response and lastly the tot number of features
    % extracted.

    %features contains the descriptors as binaryFeatures objects

%% feature matching
    % We take the keypoint's descriptors and confront them one by one until we
    % have a match. The confront is made via SSD, which is also the default
    % setting, that's why it is not specified.

    % the matching ratio is set to 0.8 which is the optimal for SIFT even if
    % we use FREAK

    indexPairs = matchFeatures(features1,features2,'MaxRatio',0.7,'Unique',true,'MatchThreshold',6.0);
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    p1 = matchedPoints1.Location;
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
    p2 = matchedPoints2.Location;

    % in matchedPoints we have 3 attributes:
    % location --> [num_matches x 2] = u,v for each match
    % metric   --> scalar index for match strength
    % count    --> num_matches
    

end
