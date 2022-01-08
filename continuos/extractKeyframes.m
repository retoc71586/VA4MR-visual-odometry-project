function [S] = extractKeyframes(S, T_w_c1, img0, img1, K)
%EXTRACTKEYFRAMES takes as input and S.p,S.X as input. 
%It must return the whole state of the current frame: S.p,S.x,S.C,S.F,S.T.

% input  :  state
%           T_w_c1 : transformation of a vector written w.r.t current
%           camera view to world frame

% Angle treshold
% need to refine threshold
if max(size(S.p)) > 600
    angleTreshold = 6;
elseif max(size(S.p)) > 500
    angleTreshold = 5;
elseif max(size(S.p)) > 400
    angleTreshold = 4;
else 
    angleTreshold = 0.5;
end

%% Traccio i keypoints nella sequenza di S.C
    S.C = round(S.C);

% Create the point tracker
    trackerKeyframes = vision.PointTracker('MaxBidirectionalError', 2, 'NumPyramidLevels', 6,'MaxIterations', 2000);
  
% Initialize the point tracker
    initialize(trackerKeyframes, S.C.', img0);
    
% Track the points
    [imagePoints1, isTracked, scores] = step(trackerKeyframes, img1);
    
%Per avere meno keypoints faccio sta cosa -Lollo
    S.C = double(imagePoints1(scores>0.8 & isTracked, :).');
    S.F = double(S.F(:, scores>0.8 & isTracked));
    S.T = S.T(:, scores>0.8 & isTracked);
    
    
% Elimino dallo stato i candidate keypoints non tracciati
%     S.C = imagePoints1(validIdx, :).';
%     S.F = S.F(: ,validIdx);
%     S.T = S.T(: ,validIdx);

    % [rotationMatrix,translationVector] = cameraPoseToExtrinsics(orientation,location)
    params.cam = cameraParameters('IntrinsicMatrix', K.');
    
    max_angolo = 30;
    S.cont = 0; %inizializzo
    for candidate_idx = 1:width(S.C)

        p_orig = [S.F(:,candidate_idx);1];
        p_curr = [S.C(:,candidate_idx);1];

        bearing_orig = K\p_orig;
        bearing_curr = K\p_curr;

        % we exploit the definition of cross and dot product, to use the more robus atan2
        angolo = atan2d(norm(cross(bearing_orig,bearing_curr)), dot(bearing_orig,bearing_curr)); 

        if angolo > max_angolo
            max_angolo = angolo;
        end

        if abs(angolo) > angleTreshold
             T_w_origin = reshape(S.T(:,candidate_idx),[4,4]);
             [R_origin_w,t_origin_w] = cameraPoseToExtrinsics(T_w_origin(1:3,1:3),T_w_origin(1:3,end));
             M0 = cameraMatrix(params.cam, R_origin_w, t_origin_w);
 
             [R_c1_w,t_c1_w] = cameraPoseToExtrinsics(T_w_c1(1:3,1:3),T_w_c1(1:3,end));
             M1 = cameraMatrix(params.cam, R_c1_w, t_c1_w);

            [NewLandmarks, reprojError] = triangulate(p_orig(1:2).',p_curr(1:2).',M0,M1);

            % newlandmark position wrt origin frame
            %NewLandmarks = linearTriangulation(p_orig,p_curr,M0.',M1.'); 
            
            dist_min = 10e100;
            for num_landm = 1:size(S.X,2)
                dist = norm(S.X(:,num_landm)-NewLandmarks');
                if dist < dist_min
                    dist_min = dist;
                end
            end
            %newLandmwrtcamera = T_w_c1(1:3,1:3).'*NewLandmarks'; %coord del landm rispetto alla camera

            if  reprojError < 1 && dist_min > 3 && norm(NewLandmarks-T_w_c1(1:3,4)) < 50000 %i punti devono stare davanti e non devono esssere troppo lontani nè dietro la fotocamera
                S.X(:,end+1) = NewLandmarks';
                S.p(:,end+1) = S.C(:,candidate_idx);
                
                S.C(:,candidate_idx) = [1, 1]';
                S.F(:,candidate_idx) = [1, 1]';

                S.cont = S.cont + 1; %contatore che mi dice quanti nuovi keypoints ho aggiunto
            end
        end

    end

%% Trovo nuovi candidate keypoints da aggiungere in S.C
% Detect feature points

    imagePoints1 = detectMinEigenFeatures(img1,'FilterSize',5,'MinQuality', 0.01);
    imagePoints1 = selectStrongest(imagePoints1,500);
    imagePoints1 = selectUniform(imagePoints1,500,size(img1));
    candidate_keypoints = imagePoints1.Location'; %2xM

    
% Ora devo aggiungere questi nuovi candidati a S.C se non sono già
% pressenti in S.C o S.p
    for new_candidate_idx = 1:width(candidate_keypoints)
        if ~ismembertol(candidate_keypoints(:,new_candidate_idx),S.C,1e-4) & ~ismembertol(candidate_keypoints(:,new_candidate_idx),S.p,1e-4)
            % we need to augment the candidate keypoint set
            S.C(:,end+1) = candidate_keypoints(:,new_candidate_idx);

            % set the first time that we have seen this new keypoints
            S.F(:,end+1) = candidate_keypoints(:,new_candidate_idx);

            % And save what was their transofrm w.r.t an interial frame called "world"
            S.T(:,end+1) = reshape(T_w_c1,[16,1]);

        else
            %fprintf('false\n');
        end
    end
