function [matchedPoints1,matchedPoints2] = matchDescriptors(valid_keyp1, valid_keyp2, features1, features2, params)

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to extract and match keypoints between two images.
% input --> the 2 images as GRAYSCALE
% output --> the matched locations in the 2 cameras frames
% Made as part of the programming assignement for Vision
% Algoritms for Mobile Robotics course, autumn 2021. ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% feature matching
    % We take the keypoint's descriptors and confront them one by one until we
    % have a match. The confront is made via SSD, which is also the default
    % setting, that's why it is not specified.

    % the matching ratio is set to 0.8 which is the optimal for SIFT even if
    % we use FREAK

    indexPairs = matchFeatures(features1,features2,'MaxRatio',0.7,'Unique',true,'MatchThreshold',6.0);
    matchedPoints1 = valid_keyp1(indexPairs(:,1),:);
    matchedPoints2 = valid_keyp2(indexPairs(:,2),:);

    % in matchedPoints we have 3 attributes:
    % location --> [num_matches x 2] = u,v for each match
    % metric   --> scalar index for match strength
    % count    --> num_matches
    

end
