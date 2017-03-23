% Fill coreDataMatrix with new calculate 2D-2D matches.
% check all counter !!!
function fill2DMatches_m(frameId1Inner, frameId2Inner, indexPairs)
    global coreDataMatrix;
       
    matchCount = size(indexPairs, 1);
    oriMPCount = coreDataMatrix.mpCount;
    mpCount = oriMPCount;
    addCount = 0;
for k = 1 : matchCount  
    firstPtMapId = coreDataMatrix.kpMpIdMatrix(indexPairs(k, 1), 2, frameId1Inner);
    if firstPtMapId == 0
            mpCount = mpCount+1;           
            coreDataMatrix.mpTrackMatrix(1 : 2, 1, mpCount) = [frameId1Inner; indexPairs(k, 1)];
            coreDataMatrix.mpTracksCountVec(mpCount, 1) = coreDataMatrix.mpTracksCountVec(mpCount, 1) + 1;
            coreDataMatrix.kpMpIdMatrix(indexPairs(k, 1), 1 : 2, frameId1Inner) =  [1, mpCount]; 
            firstPtMapId = mpCount;
            addCount = addCount + 1;
    end
    
    if coreDataMatrix.kpMpIdMatrix(indexPairs(k, 2), 2, frameId2Inner) ~= 0
        % disp('del')
        delMpTrackConnect_m(indexPairs(k, 2), frameId2Inner);
    end
    
    % check duplication
    isDup = false;
    for j = 1 : coreDataMatrix.mpTracksCountVec(firstPtMapId, 1);
        if coreDataMatrix.mpTrackMatrix(1, j, firstPtMapId) == frameId2Inner
                display('[FillMatches] Has duplicated frameID in a Track!!');
                isDup = true;
        end
    end
    if isDup
        continue;
    end
    
    %connect mp and kp
    tracksLen = coreDataMatrix.mpTracksCountVec(firstPtMapId, 1);
    coreDataMatrix.mpTrackMatrix(:, tracksLen + 1, firstPtMapId) = [frameId2Inner; indexPairs(k, 2)]; 
    coreDataMatrix.mpTracksCountVec(firstPtMapId, 1) = coreDataMatrix.mpTracksCountVec(firstPtMapId, 1) + 1;
    
    % add MpId of Frame2
    currMpPosi = coreDataMatrix.kpMpIdMatrix(indexPairs(k, 2), 1, frameId2Inner) + 1;
    coreDataMatrix.kpMpIdMatrix(indexPairs(k, 2), currMpPosi + 1, frameId2Inner) = firstPtMapId;
    coreDataMatrix.kpMpIdMatrix(indexPairs(k, 2), 1, frameId2Inner) = ...
        coreDataMatrix.kpMpIdMatrix(indexPairs(k, 2), 1, frameId2Inner) + 1;  
    
    % merge descriptor
    descSet = uint8(zeros(size(coreDataMatrix.mpDescMatrix, 1), coreDataMatrix.mpTracksCountVec(firstPtMapId, 1)));
    connect = coreDataMatrix.mpTrackMatrix(:, 1:coreDataMatrix.mpTracksCountVec(firstPtMapId, 1), firstPtMapId);

    for idx = 1 : coreDataMatrix.mpTracksCountVec(firstPtMapId, 1)
        descSet(:, idx) = coreDataMatrix.kpDescMatrix(:, connect(2, idx), connect(1, idx));
    end
    coreDataMatrix.mpDescMatrix(:, firstPtMapId) = (MergeDescriptor_m(descSet'))';

end

    % update the mpCount in the end
    coreDataMatrix.mpCount  = coreDataMatrix.mpCount + addCount; 

end
    


    
    