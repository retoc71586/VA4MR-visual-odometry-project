function T = extractGroundTruth(poses, reference_frame_number, final_frame_number)
%EXTRACTGROUNDTRUTH takes the poses.txt fil as input and ouputs the
%transofrmation matrix from reference_frame to _last_frame: T 
    ref_pose_row = poses(reference_frame_number,:);
    ref_pose = reshape(ref_pose_row,[4,3]);
    ref_pose = ref_pose';
    
    last_pose_row = poses(final_frame_number,:);
    last_pose = reshape(last_pose_row,[4,3]);
    last_pose = last_pose';
    
    T = [ref_pose; 0 0 0 1]\[last_pose; 0 0 0 1];
    T = T(1:3,:);
end

