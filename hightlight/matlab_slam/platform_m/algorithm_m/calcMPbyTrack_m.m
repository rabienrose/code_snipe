function isSucc = calcMPbyTrack_m(frameId, offset)
    
    global params;
    global coreDataMatrix;
    
    %check boundary.
    frameIdInner = coreDataMatrix.frImgIdVec(frameId, 1);  % frameId
    frameIdInnerPre = frameIdInner + offset;  % frameIdl
    
    kfCount = size(coreDataMatrix.kpsCountVec, 1);
    if frameIdInnerPre > kfCount || frameIdInnerPre < 1
        isSucc = false;
        return;
    end
    
    if offset == 0
        return;
    end
    
    %% find tracks those map point has none position.
    kpCount = coreDataMatrix.kpsCountVec(frameIdInner, 1);
    keypoint = struct('Location', [],...
                      'Scale', [],...
                      'Metric', [],...
                      'Octave', []);

    matchedPointsl = keypoint;
    matchedPointsl.Location(kpCount, 2) = 0;
    matchedPointsl.Scale(kpCount, 1) = 0;
    matchedPointsl.Metric(kpCount, 1) = 0;
    matchedPointsl.Octave(kpCount, 1) = 0;
    
    matchedPointsc = keypoint;
    matchedPointsc.Location(kpCount, 2) = 0;
    matchedPointsc.Scale(kpCount, 1) = 0;
    matchedPointsc.Metric(kpCount, 1) = 0;
    matchedPointsc.Octave(kpCount, 1) = 0;
    
    mpIds(kpCount, 1) = 0;
    ptCount =0;
    for i = 1 : kpCount
        mapId = coreDataMatrix.kpMpIdMatrix(i, 2, frameIdInner);
        if mapId ~= 0
            % default position value is NaN
            if sum(isnan(coreDataMatrix.mpPosiMatrix(:, mapId))) ~= 3 
                continue;
            end
            %--------- get the Matched kpId in DataRootMatrix--------
            %--------------- previous " getMatchedPt() "  -------------
            currTrack = coreDataMatrix.mpTrackMatrix(:, :, mapId);
            [~, col] = find(currTrack(1, :) == frameIdInnerPre);
            if isempty(col)
%                 disp('This frameId is not in the track!');
                kpInd = -1;
            else
                kpInd = currTrack(2, col);
            end 
            %--------------------------------------------------------
            if kpInd ~= -1
                
                kpIdl = kpInd;
                
                ptCount= ptCount+1;
                
                % location -> row                
                matchedPointsl.Location(ptCount, :) = coreDataMatrix.kpPosiMatrix(:, kpIdl, frameIdInnerPre)';             
                matchedPointsl.Octave(ptCount, 1) = coreDataMatrix.kpOctaveMatrix(kpIdl, frameIdInnerPre);
                matchedPointsl.Scale(ptCount, 1) = 1;
                matchedPointsl.Metric(ptCount, 1) = 1;
                
                matchedPointsc.Location(ptCount, :) = coreDataMatrix.kpPosiMatrix(:, i, frameIdInner)';             
                matchedPointsc.Octave(ptCount, 1) = coreDataMatrix.kpOctaveMatrix(i, frameIdInner);
                matchedPointsc.Scale(ptCount, 1) = 1;
                matchedPointsc.Metric(ptCount, 1) = 1;
                mpIds(ptCount, 1) = mapId;
            end
        end
    end
    
    if ptCount<=20
        isSucc = false;
        return;
    end
    
    matchedPointsc.Location = single(matchedPointsc.Location(1: ptCount, :));
    matchedPointsc.Octave = uint32(matchedPointsc.Octave(1: ptCount, :));
    matchedPointsc.Scale = single(matchedPointsc.Scale(1: ptCount, :));
    matchedPointsc.Metric = single(matchedPointsc.Metric(1: ptCount,:));
    matchedPointsl.Location = single(matchedPointsl.Location(1: ptCount, :));
    matchedPointsl.Octave = uint32(matchedPointsl.Octave(1: ptCount, :));
    matchedPointsl.Scale = single(matchedPointsl.Scale(1: ptCount, :));
    matchedPointsl.Metric = single(matchedPointsl.Metric(1: ptCount, :));
    mpIds= mpIds(1: ptCount, 1);
    
    
    posel = coreDataMatrix.frPoseMatrix(:, :, frameIdInnerPre);
    posec = coreDataMatrix.frPoseMatrix(:, :, frameIdInner);

    %% triangulate using tracks.
    [xyzPoints, successFlag] = Triangulate_m(...
                            matchedPointsl, posel,...
                            matchedPointsc, posec,...
                            params.cameraParam,...
                            params.scaleFactorSet, params.ORB_scaleFactor);
                     
    %% update map point list.                    
    n = 0;
    for i = 1 : size(successFlag, 1)
        if successFlag(i) ~= 1
            continue;
        end
        n = n + 1;
        coreDataMatrix.mpPosiMatrix(:, mpIds(i)) = xyzPoints(n, :)';
    end
    
    if n == 0
        isSucc = false;
        return;
    end
    isSucc = true;
end



