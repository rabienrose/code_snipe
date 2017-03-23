function [isSucc] = matchByProjectionCore_m(frameId, frameCount)

    global coreDataMatrix;
    global params;
    
    if(frameId <= 0 || frameId > params.imagePaths.Count)
        disp('invalid input in matchByProjectionCore, please check!');
        isSucc = false;
        return;
    end
    
    frameIdInner = coreDataMatrix.frImgIdVec(frameId, 1);
    
    %check boundary. frCount should be replace with keyfrCount later (when frTypeMatrix is ready)
    if(frameIdInner <= 1 || frameIdInner > size(coreDataMatrix.kpsCountVec, 1) || frameCount <= 0)
        disp('invalid input in matchByProjectionCore, please check!');
        isSucc = false;
        return;
    end
            
    kpsCountAll = zeros(frameCount, 1);
    for j = 1 : frameCount
        if (frameIdInner - j) < 1
            break;
        end
        kpsCountAll(j) = coreDataMatrix.kpsCountVec(frameIdInner - j, 1);
    end
    kpsCountAllNum = sum(kpsCountAll);
    counteri = 0;
    dim = size(coreDataMatrix.mpDescMatrix, 1);
    point3D = zeros(kpsCountAllNum, 3);
    descMp = zeros(kpsCountAllNum, dim);
    mpExistList = zeros(kpsCountAllNum, 1);
    for j = 1 : frameCount
        if (frameIdInner - j) < 1
            break;
        end
        kpsCounti = kpsCountAll(j);
        for i = 1 : kpsCounti
            % only use the first mpId here
            mpId = coreDataMatrix.kpMpIdMatrix(i, 2, frameIdInner - j);
            if (mpId == 0) || ~isempty(find(mpExistList == mpId, 1))
               continue; 
            end
            mpPoint = coreDataMatrix.mpPosiMatrix(:, mpId)';
            if isempty(mpPoint);
               continue; 
            end
            mpDesc = coreDataMatrix.mpDescMatrix(:, mpId)';
            counteri = counteri + 1;
            
            % change here when pass test
            point3D(counteri, :) = mpPoint;
            descMp(counteri, :) = mpDesc;
            mpExistList(counteri, 1) = mpId;
        end
    end
    
    mpExistList = mpExistList(1 : counteri, 1);
    
    point3D = single(point3D(1 : counteri, :));
    descMp = uint8(descMp(1 : counteri, :));
    
    poseCurr = coreDataMatrix.frPoseMatrix(:, :, frameIdInner);
    K = params.cameraParam;
    imageSize = [0, 0, coreDataMatrix.imgSize(1, 1), coreDataMatrix.imgSize(1, 2)];
    num = coreDataMatrix.kpsCountVec(frameIdInner);
    pts = zeros(num, 2);
%     dim = size(coreDataMatrix.mpDescMatrix, 1);
    desc = zeros(num, dim);
    idMapper(num,1) =0;
    count =0;
    for i = 1 : num
        if coreDataMatrix.kpMpIdMatrix(i, 2, frameIdInner) ~= 0
            continue;
        end
        count = count + 1;
        pts(count, :) = coreDataMatrix.kpPosiMatrix(:, i, frameIdInner)';
        desc(count, :) = coreDataMatrix.kpDescMatrix(:, i, frameIdInner)';
        idMapper(count, 1) = i;
    end
    pts = pts(1 : count, :);
    desc = desc(1 : count, :);
    idMapper = idMapper(1 : count, :);
    mapPoints = struct('MapPoint', point3D, 'Descriptor', descMp);
    keys = struct('KeyPoint', pts, 'Descriptor', desc);
    [matchedPairs, ~, ~] =...
        searchByProjection_m(mapPoints, poseCurr, K, imageSize, keys);
    
    % Updata KF: mpId, trackItemId; MP: frameIdInner, KPId
    for i = 1 : size(matchedPairs, 1)
        mpIdx = mpExistList(matchedPairs(i, 1));
        ptIdx = idMapper(matchedPairs(i, 2));
        
        if  coreDataMatrix.kpMpIdMatrix(ptIdx, 2, frameIdInner) ~= 0   
            % updata the MpId and corrsponding count (only process the 1st MpId)
            coreDataMatrix.kpMpIdMatrix(ptIdx, 2, frameIdInner) = 0;
            % !!! Notice: Here we set count as 0, may be modified later.(since here only process the 1st MpId)
            coreDataMatrix.kpMpIdMatrix(ptIdx, 1, frameIdInner) = 0;
            delMpTrackConnect_m(ptIdx, frameIdInner);
        end
        
        % check duplication
        isDup = false;
        for j = 1 : coreDataMatrix.mpTracksCountVec(mpIdx, 1);
            if coreDataMatrix.mpTrackMatrix(1, j, mpIdx) == frameIdInner
%                 display('[MatchByProjectionCore] Has duplicated frameIdInner in a Track!!');
                isDup = true;
                break;
            end
        end
        if isDup
            continue;
        end
              
        % connect mp and kp
        % add MpId 
        currMpPosi = coreDataMatrix.kpMpIdMatrix(ptIdx, 1, frameIdInner) + 2;
        coreDataMatrix.kpMpIdMatrix(ptIdx, currMpPosi, frameIdInner) = mpIdx;
        coreDataMatrix.kpMpIdMatrix(ptIdx, 1, frameIdInner) = coreDataMatrix.kpMpIdMatrix(ptIdx, 1, frameIdInner) + 1; 
        
        % add track
        tracksLen = coreDataMatrix.mpTracksCountVec(mpIdx, 1);
        coreDataMatrix.mpTrackMatrix(:, tracksLen + 1, mpIdx) = [frameIdInner; ptIdx]; 
        coreDataMatrix.mpTracksCountVec(mpIdx, 1) = coreDataMatrix.mpTracksCountVec(mpIdx, 1) + 1;
              
        % merge descriptor
        trackNum = coreDataMatrix.mpTracksCountVec(mpIdx, 1);
        descSet = uint8(zeros(dim, trackNum));
        connect = coreDataMatrix.mpTrackMatrix(:, 1:trackNum, mpIdx);
        for idx = 1 : trackNum
            descSet(:, idx) = coreDataMatrix.kpDescMatrix(:, connect(2, idx), connect(1, idx));
        end
        coreDataMatrix.mpDescMatrix(:, mpIdx) = (MergeDescriptor_m(descSet'))';  
        
    end
    isSucc = true;
end



