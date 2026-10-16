function [X, Y, yawDeg] = lka_pose_to_sim3d(pose, frameMap)
% LKA_POSE_TO_SIM3D Track pose (OpenDRIVE / RoadRunner frame) -> Simulation 3D inputs.
%
% Syntax:
%   [X, Y, yawDeg] = lka_pose_to_sim3d(pose, frameMap)     pose = [x y psi] (m, m, rad)
%
% frameMap 2 is the identity (Simulation 3D Vehicle with Ground Following in ISO 8855
% world coordinates); frameMap 3 is the convention of the aBaja2026.slx InitFcn
% (RoadRunner -> Unreal: initialPosition = pos.*[1 -1 1], yaw negated); 1 and 4 are the
% remaining candidates tested by calibrate_vision_bench.
    x = pose(1); y = pose(2); psiDeg = rad2deg(pose(3));
    switch frameMap
        case 1
            X = -y; Y = -x; yawDeg = -90 - psiDeg;
        case 2
            X = x;  Y = y;  yawDeg = psiDeg;
        case 3
            X = x;  Y = -y; yawDeg = -psiDeg;
        otherwise
            X = y;  Y = x;  yawDeg = 90 - psiDeg;
    end
end
