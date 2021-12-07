function [T, keypoints_img0, keypoints_img1, landmarks] = twoWiewSFM(img0,img1,K)
    figures = 0;
    % schange numiter to verify accuracy of SFM
    numiter = 1;
    
    for i = 1:numiter
    if i > 1
       fprintf(i);
    end
    % Detect feature points
    imagePoints0 = detectMinEigenFeatures(img0, 'MinQuality', 0.1);

    if figures
        % Visualize detected points
        figure
        imshow(img0, 'InitialMagnification', 50);
        title('200 Strongest Corners from the First Image');
        hold on
        plot(selectStrongest(imagePoints0, 200));
    end
    
    % Create the point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

    % Initialize the point tracker
    p0 = imagePoints0.Location;
    initialize(tracker, p0, img0);

    % Track the points
    [imagePoints2, validIdx] = step(tracker, img1);
    matchedPoints0 = p0(validIdx, :);
    matchedPoints1 = imagePoints2(validIdx, :);

    if figures
        % Visualize correspondences
        figure
        showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
        title('Tracked Features');
    end
    
    % Estimate the fundamental matrix
    [F, inliers] = estimateFundamentalMatrix(matchedPoints0, matchedPoints1,'Confidence', 99.99);
    E = K' * F * K;
    [R,u] = decomposeEssentialMatrix(E);
    num_keyp = size(matchedPoints0(inliers,:),1);
    p0_ho = [matchedPoints0(inliers,:), ones(num_keyp,1)]';
    p1_ho = [matchedPoints1(inliers,:), ones(num_keyp,1)]';
    [R,t] = disambiguateRelativePose(R,u,p0_ho,p1_ho,K,K);

    % Find epipolar inliers
    keypoints_img0 = matchedPoints0(inliers, :);
    keypoints_img1 = matchedPoints1(inliers, :);

    if figures
        % Display inlier matches
        figure
        showMatchedFeatures(img0, img1, keypoints_img0, keypoints_img1);
        title('Epipolar Inliers');
    end
    
    %triangulate points
    T = [R, t];
    landmarks = pointCloud(img0, p0_ho, p1_ho, K, T, 0);
    
    end
end