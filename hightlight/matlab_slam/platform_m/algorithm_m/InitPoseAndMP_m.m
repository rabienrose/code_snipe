function [isSucc] = InitPoseAndMP(frameId)
%     addpath('../AlgoLib');
%     addpath('../Algolib/Triangulate');
%     addpath('../Algolib/MergeDescriptor');
%     addpath('../Visualization');

    global params;
    
    % check boundary
    if(frameId + 1 > params.imagePaths.Count)
        isSucc = false;
        return;
    end
    
    % calcualte features.
    CalFeaturesOneFrame(frameId);
    CalFeaturesOneFrame(frameId + 1);
    
    % matching using optical flow.
    MatchByOptiFlowCore(frameId + 1, -1);
    
    figure(1);
    ShowTrack(frameId + 1);
    
    % calculate pose using matches.
    CalRTby2Frame(frameId + 1, -1);
    
    % generate map points.
    CalMPbyTrack(frameId + 1, -1);
    
    figure(2);
    ShowReprojection(frameId + 1);
    
    isSucc = true;
end

