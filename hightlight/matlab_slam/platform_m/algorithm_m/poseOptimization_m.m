% Optimize the rough pose

function [isSucc] = poseOptimization_m(frameId)
    global params;
    global coreDataMatrix;
    
    if frameId <= 1 || frameId > params.imagePaths.Count
        disp('invalid input in PoseOptimization, please check!');
        isSucc = false;
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner <= 1
        disp('invalid inner input in PoseOptimization, please check!');
        isSucc = false;
        return;
    end
    
    posec = coreDataMatrix.frPoseMatrix(:, :, currIdInner);
    
    mpIdCount = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner);
    nonZeroIdx = mpIdCount ~= 0;
    if sum(nonZeroIdx) == 0
        disp('empty map points for pose optimization.');
        isSucc = false;
        return;
    end
    mpIdSet = coreDataMatrix.kpMpIdMatrix(nonZeroIdx, 2, currIdInner);
    matchedMps = coreDataMatrix.mpPosiMatrix(:, mpIdSet);
    matchedKps = coreDataMatrix.kpPosiMatrix(:, nonZeroIdx, currIdInner);
    keys_octave = coreDataMatrix.kpOctaveMatrix(nonZeroIdx, currIdInner);
    checkMpsIdx = isnan(matchedMps(1, :));
    if isempty(find(checkMpsIdx == 0, 1))
       isSucc = false;
       return; 
    end
    matchedMps(:, checkMpsIdx) = []; % mps
    matchedKps(:, checkMpsIdx) = []; % kps
    keys_octave(checkMpsIdx, :) = []; % octave
%     nonZeroIdx(checkMpsIdx, :) = 0; % used kps index
    
    idx1 = find(nonZeroIdx == 1);
    idx2 = find(checkMpsIdx == 0)';
    
    currKeys = struct('Location', [], 'Scale', [], 'Metric', [], 'Misc', [], 'Orientation', [], 'Octave', []);
    currKeys.Location       = single(matchedKps');
    currKeys.Scale          = single(ones(size(currKeys.Location, 1), 1));
    currKeys.Metric         = single(zeros(size(currKeys.Location, 1), 1));
    currKeys.Misc           = cast(zeros(size(currKeys.Location, 1), 1), 'uint32');
    currKeys.Orientation    = single(zeros(size(currKeys.Location, 1), 1));
    currKeys.Octave         = cast(keys_octave, 'uint32');
    
    [optimizedPose3D, outliers] = ...
                                OptimizePose_m(params.cameraParam, ...
                                             posec, ...
                                             matchedMps', ...
                                             currKeys, ...
                                             params.scaleFactorSet);
                                         
    % when 2D-3D pairs < 3, optimizedPose3D is empty
    if isempty(optimizedPose3D)
        isSucc = false;
        return;
    end
    
    coreDataMatrix.frPoseMatrix(:, :, currIdInner) = double(optimizedPose3D(1:3,:));
    
    % Delete outliers
    idx3 = outliers == 1;
    deleteIdx = idx1(idx2(idx3));
    delMpTrackConnect_m(deleteIdx, currIdInner);
    
    isSucc = true;
end




