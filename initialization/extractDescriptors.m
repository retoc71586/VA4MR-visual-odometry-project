function [features1,features2,valid_points1,valid_points2] = extractDescriptors(img1, img2, keypoints1, keypoints2, params)
%% feature extraction
    % After detecting the corners we need to extract the descriptors in order to
    % do the matching, extractFeatures does that.
    % The type of descriptor returned depends on the type of points1. In this
    % case it is a cornerPoints so we get FREAK descriptors, fast to compute and
    % both rotation and scale invariant.
    % we could set upright to true since the images are not rotated in parking
    % but this way we can use it for all the datasets; NB: rotation in radiants

    %%%%%% comunque SIFT è overshoot 
    [features1,valid_points1] = extractFeatures(img1,keypoints1, 'Method', 'Block', 'BlockSize', params.descriptor_size);
    [features2,valid_points2] = extractFeatures(img2,keypoints2, 'Method', 'Block', 'BlockSize', params.descriptor_size);

    % valid points is a cornerPoints object containing for each feature the px
    % location, cornerness response and lastly the tot number of features
    % extracted.

    %features contains the descriptors as binaryFeatures objects
end

