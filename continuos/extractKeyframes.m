function [S] = extractKeyframes(S, T_w_c1, img0, img1, K)
%EXTRACTKEYFRAMES takes as input and S.p,S.X as input. 
%It must return the whole state of the current frame: S.p,S.x,S.C,S.F,S.T.

% input  :  state
%           T_w_c1 : transformation of a vector written w.r.t current
%           camera view to world frame

% Angle treshold
% need to refine threshold
    angleTreshold = 5;

%% Traccio i keypoints nella sequenza di S.C
% Create the point tracker
    trackerKeyframes = vision.PointTracker('MaxBidirectionalError', 2, 'NumPyramidLevels', 6);
  
% Initialize the point tracker
    initialize(trackerKeyframes, S.C.', img0);
    
% Track the points
    [imagePoints1, validIdx] = step(trackerKeyframes, img1);

% Elimino dallo stato i candidate keypoints non tracciati
    S.C = imagePoints1(validIdx, :).';
    S.F = S.F(: ,validIdx);
    S.T = S.T(: ,validIdx);

    % [rotationMatrix,translationVector] = cameraPoseToExtrinsics(orientation,location)
    params.cam = cameraParameters('IntrinsicMatrix', K.');
    
    max_angolo = 0;
    num_keypoints_aggiunti = 0;

    for candidate_idx = 1:width(S.C)

        p_orig = [S.F(:,candidate_idx);1];
        p_curr = [S.C(:,candidate_idx);1];

        bearing_orig = K\p_orig;
        bearing_curr = K\p_curr;

        % we exploit the definition of cross and dot product, to use the more robus atan2
        angolo = atan2d(norm(cross(p_orig,p_curr)), dot(p_orig,p_curr))*180/pi; 

        if angolo > max_angolo
            max_angolo = angolo;
        end

        if abs(angolo) > angleTreshold
            num_keypoints_aggiunti = num_keypoints_aggiunti +1;

            T_w_origin = reshape(S.T(:,candidate_idx),[4,4]);
            [R_w_origin,t_w_origin] = cameraPoseToExtrinsics(T_w_origin(1:3,1:3),T_w_origin(1:3,end));
            M0 = cameraMatrix(params.cam, R_w_origin, t_w_origin);

            [R_w_c1,t_w_c1] = cameraPoseToExtrinsics(T_w_c1(1:3,1:3),T_w_c1(1:3,end));
            M1 = cameraMatrix(params.cam, R_w_c1, t_w_c1);

            %NewLandmarks = triangulate(p_orig,p_curr,M0,M1);

            % newlandmark position wrt origin frame
            NewLandmarks = linearTriangulation(p_orig,p_curr,M0.',M1.'); 
            
            S.X(:,end+1) = NewLandmarks(1:3,:);
            S.p(:,end+1) = S.C(:,candidate_idx);
        end
    end


%% Trovo nuovi candidate keypoints da aggiungere in S.C
% Detect feature points
    imagePoints1 = detectMinEigenFeatures(img1,'MinQuality', 0.01);
    imagePoints1 = selectStrongest(imagePoints1,1000);
    imagePoints1 = selectUniform(imagePoints1,50,size(img1));

    candidate_keypoints = imagePoints1.Location'; %2xM
% Ora devo aggiungere questi nuovi candidati a S.C sse non sonon già
% pressenti in S.C o S.p
    for new_candidate_idx = 1:width(candidate_keypoints)
        if ~ismember(candidate_keypoints(:,new_candidate_idx),S.C) & ~ismember(candidate_keypoints(:,new_candidate_idx),S.p)
            % we need to augment the candidate keypoint set
            S.C(:,end+1) = candidate_keypoints(:,new_candidate_idx);

            % set the first time that we have seen this new keypoints
            S.F(:,end+1) = candidate_keypoints(:,new_candidate_idx);

            % And save what was their transofrm w.r.t an interial frame called "world"
            S.T(:,end+1) = reshape(T_w_c1,[16,1]);

        else
            fprintf('false\n');
        end
    end
