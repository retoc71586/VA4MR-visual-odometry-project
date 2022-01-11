function [strongest1,strongest2] = extractKeypoints(I1,I2,params)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to extract and match keypoints between two images.
% input --> the 2 images as GRAYSCALE and the method to be used
% output --> the extracted keypoints in both images
% Made as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figures = false;
%0 = Harris, 1 = Surf; %2 Shi-Tomasi
method = params.method;
%% corner detection
if method == 0
    points1 = detectHarrisFeatures(I1,'FilterSize',params.filt_size,'MinQuality', params.feature_quality); % points1 is a cornerPoints object
    points2 = detectHarrisFeatures(I2,'FilterSize',params.filt_size,'MinQuality', params.feature_quality);
elseif method == 1
    points1 = detectSURFFeatures(I1,'MetricThreshold',200);
    points2 = detectSURFFeatures(I2,'MetricThreshold',200);
elseif method == 2
    points1 = detectMinEigenFeatures(I1,'FilterSize',params.filt_size,'MinQuality', params.feature_quality); % points1 is a cornerPoints object
    points2 = detectMinEigenFeatures(I2,'FilterSize',params.filt_size,'MinQuality', params.feature_quality);
end

strongest1 = selectStrongest(points1,1000); % selectStrongest is a method of cornerPoints
strongest2 = selectStrongest(points2,1000);

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
end

