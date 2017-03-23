function [isSucc] = calcRTby2Frame_m(frameId, offset)
  
    global params;
    global coreDataMatrix;
    
    if offset ==0
        isSucc = false;
        return;
    end
    
    frameIdInner = coreDataMatrix.frImgIdVec(frameId, 1);
    frameIdInnerPre = frameIdInner + offset;
        
    % initialize matched points for F computation
    kpCount = coreDataMatrix.kpsCountVec(frameIdInner);
    matchedPoints1(kpCount, 2) = 0;
    matchedPoints2(kpCount, 2) = 0;
    idMapper1(kpCount) = 0;
    idMapper2(kpCount) = 0;
    count = 0;
    
    % find matches 
    for i = 1 : kpCount
        % Notice: here we only use the 1st mpId 
        mpId = coreDataMatrix.kpMpIdMatrix(i, 2, frameIdInner);
        mpCountFlag = coreDataMatrix.kpMpIdMatrix(i, 1, frameIdInner);
        if (mpCountFlag == 0 || mpId == 0) 
            continue;
        end
        
        %--------- get the Matched kpId in DataRootMatrix--------
        %--------------- previous " getMatchedPt() "  -------------
        currTrack = coreDataMatrix.mpTrackMatrix(:, :, mpId);
        [~, col] = find(currTrack(1, :) == frameIdInnerPre);
        if isempty(col)
%             disp('This frameId is not in track!');
            kpInd = -1;
        else
            kpInd = currTrack(2, col);
        end 
        %--------------------------------------------------------
        
        if kpInd == -1
            continue;
        end
        count = count+1;
        kpIdPre = kpInd;
        matchedPoints1(count, :) = coreDataMatrix.kpPosiMatrix(:, kpIdPre, frameIdInnerPre)';
        matchedPoints2(count, :) = coreDataMatrix.kpPosiMatrix(:, i, frameIdInner)';
        idMapper1(count) = kpIdPre;
        idMapper2(count) = i;
    end
    matchedPoints1 = matchedPoints1(1 : count, :);
    matchedPoints2 = matchedPoints2(1 : count, :);
    
    % if the matches is too small, just return
    %%  _________ should be modify here when testing over______
    if count <= 20
        isSucc = false;
        return;
    end
    
    % cal F
    [F, epipolarInliers] = estimateFundamentalMatrix(...
            matchedPoints1, matchedPoints2, 'Method', 'MSAC', 'NumTrials', 10000);

    %------ del connection of the outlier in coreDataMatrix--------
    %------------ previous " DelMPKPConnection() "  ---------------
    % find outliers' index 
    OutlierRow = find(epipolarInliers == 0);
    outKp1Set = idMapper1(OutlierRow);
    outKp2Set = idMapper2(OutlierRow);
    % delete the connections; Notice: here we only use the 1st mpId 
    if ~isempty(outKp1Set)
        % delete MpId
        num1 = size(outKp1Set, 2);
        coreDataMatrix.kpMpIdMatrix(outKp1Set, 1 : 2, frameIdInnerPre)...
                = [zeros(num1, 1), 0 * ones(num1, 1)];
            
        % delete Tracks   
        delMpTrackConnect_m(outKp1Set, frameIdInnerPre);
    end
    if ~isempty(outKp2Set)
        % delete MpId
        num2 = size(outKp2Set, 2);
        coreDataMatrix.kpMpIdMatrix(outKp2Set, 1 : 2, frameIdInner)...
                = [zeros(num2, 1), 0 * ones(num2, 1)];
        % delete Tracks     
        delMpTrackConnect_m(outKp2Set, frameIdInner);
    end
    %--------------------------------------------------------------    
    
    % cal R, T from F
    inlierPoints1 = matchedPoints1(epipolarInliers, :);
    inlierPoints2 = matchedPoints2(epipolarInliers, :);
    cameraParams = cameraParameters('IntrinsicMatrix', params.cameraParam');
    [R, t] = cameraPose(F, cameraParams, inlierPoints1, inlierPoints2);
    coreDataMatrix.frPoseMatrix(:, :, frameIdInner - 1) = [eye(3,3), [0,0,0]'];
    tPose = [R', t'; 0 0 0 1]^-1;
    coreDataMatrix.frPoseMatrix(:, :, frameIdInner) = tPose(1 : 3, :);
    isSucc= true;
    
end



