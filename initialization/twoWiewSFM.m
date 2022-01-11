function [T, matchedPoints1, landmarks] = twoWiewSFM(img0,img1,params)

    figures = 0;
    % Detect feature points
    %imagePoints0 = detectMinEigenFeatures(img0, 'MinQuality', 0.1);
    imagePoints0 = detectHarrisFeatures(img0, 'MinQuality', params.feature_quality, 'FilterSize', params.filt_size);
    imagePoints0 = selectStrongest(imagePoints0, params.n_keypoints);
    if figures
        % Visualize detected points
        figure(1)
        imshow(img0, 'InitialMagnification', 50);
        title('Strongest Corners from the First Image');
        hold on
        plot(imagePoints0);
        hold off
    end
    
    % Create the point tracker
    tracker = vision.PointTracker('MaxBidirectionalError', params.lambda, ...
                                   'NumPyramidLevels', params.num_pyr_levels, ...
                                   'BlockSize', params.bl_size, ...
                                   'MaxIterations', params.max_its);

    % Initialize the point tracker
    p0 = imagePoints0.Location;
    p0 = round(p0);
    initialize(tracker, p0, img0);

    % Track the points
    [imagePoints2, validIdx] = step(tracker, img1);
    matchedPoints0 = p0(validIdx, :);
    imagePoints2 = round(imagePoints2);
    matchedPoints1 = imagePoints2(validIdx, :);

    % Estimate the fundamental matrix
    [F, inliers] = estimateFundamentalMatrix(matchedPoints0, matchedPoints1,'Confidence', 99.99,'Method','Ransac');

    matchedPoints0 = matchedPoints0(inliers,:);
    matchedPoints1 = matchedPoints1(inliers,:);
    for i = 1:max(size(matchedPoints0))
        track(i) = pointTrack([1 2],[matchedPoints0(i,:);matchedPoints1(i,:)]);
    end

    % calc the pose of the camera 1 as seen from the camera 0
    [R,t,validPointsFraction] = relativeCameraPose(F, params.cam, matchedPoints0, matchedPoints1);

    if validPointsFraction < 0.5
    warning('[relativeCameraPose] ERROR: relative pose is invalid %f', validPointsFraction);
    end

    [R_I_w,t_I_w] = cameraPoseToExtrinsics(eye(3),[0 0 0]);
    M0 = cameraMatrix(params.cam, R_I_w, t_I_w);
    
    [R_c1_w,t_c1_w] = cameraPoseToExtrinsics(R,t);
    M1 = cameraMatrix(params.cam, R_c1_w, t_c1_w + t_I_w);
    
    Orientation = {R_I_w';R_c1_w'};
    Location = {-t_I_w; -t_c1_w};
    ViewId = uint32([1;2]);
        
    Table = table(ViewId, Orientation,Location);
    
    [landmarks, reprojError] = triangulate(matchedPoints0,matchedPoints1,M0,M1);
    
    [landmarks,T_n] = bundleAdjustment(landmarks,track,Table,params.cam);
    %M0 = cameraMatrix(params.cam, T_n.Orientation{1}, T_n.Location{1});
    %M1 = cameraMatrix(params.cam, T_n.Orientation{2}, T_n.Location{2});
   
    T(1:3,1:3) = T_n.Orientation{2};
    T(1:3,4) = T_n.Location{2};
    T(4,:) = [0 0 0 1];
    
    landmarks = landmarks.';


    if figures
        % Display inlier matches
        figure(2)
        showMatchedFeatures(img0, img1, matchedPoints0, matchedPoints1);
        title('Epipolar Inliers');
        figure(3)
        plot3(landmarks(1,:),landmarks(2,:),landmarks(3,:),'o'); hold on
        title('Landmarks');
        
        
        plotCoordinateFrame(T(1:3,1:3),T(1:3,4), 80);
        
        axis equal
        rotate3d on;
    end
    
    
    %landmarks = pointCloud(img0, p0_ho, p1_ho, K, T, 0);

end