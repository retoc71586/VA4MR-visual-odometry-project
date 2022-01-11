function [s] = CalcScalingFactor(ds,img_num, T_w_c0, T_first,s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to calculate scaling
% Made by Roberto Pellerito as part of the programming assignement
% for Vision Algoritms for Mobile Robotics course, autumn 2021. 
% ETH Zurich
%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ds == 2 && img_num == 105
    %parking
        % the average width of a land rover is 1.8 meters
        relative_trasl = T_w_c0(1:3,end) - T_first(1:3,end);
        rel_trasf_frames = relative_trasl / 1.8;
        % ideally should be 1
        s = max(rel_trasf_frames/5);

elseif ds == 1 && img_num == 99
    %Malaga
    % it sees a car at frame 95
        % the average lenght of a Seat ibiza is 4.059 meters
        relative_trasl = T_w_c0(1:3,end) - T_first(1:3,end);
        rel_trasf_frames = relative_trasl / 4.059;
        % ideally should be 1
        s = max(rel_trasf_frames/4);

elseif ds == 0 && img_num == 43
    %Kitti
        % the average width of a Ww wagon is 2 meters
        relative_trasl = T_w_c0(1:3,end) - T_first(1:3,end);
        rel_trasf_frames = relative_trasl / 2;
        % ideally should be 1
        s = max(rel_trasf_frames/3);
else 
    s = s;
end




end