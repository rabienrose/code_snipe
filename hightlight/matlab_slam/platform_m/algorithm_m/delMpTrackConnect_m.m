function delMpTrackConnect_m(OutKpSet, frameIdInner)
    global coreDataMatrix;
    for Idx = 1 : size(OutKpSet, 1)
        outKp = OutKpSet(Idx, 1);
        mpCountFlag = coreDataMatrix.kpMpIdMatrix(outKp, 1, frameIdInner);
        mpId = coreDataMatrix.kpMpIdMatrix(outKp, 2, frameIdInner);
        if mpCountFlag <= 0 || mpId == -1
            continue;
        end
        
        % delete the frameId & KpId in mpTrackMatrix
        currTrackLen = coreDataMatrix.mpTracksCountVec(mpId, 1);
        trackCurrMpSet = coreDataMatrix.mpTrackMatrix(:, 1:currTrackLen, mpId);
        [~, col] = find(trackCurrMpSet(1, :) == frameIdInner);
        
        % delete the current colomn (frameId, KpId), and save new result 
        if ~isempty(col)
            trackCurrMpSet = [trackCurrMpSet(:, [1 : col - 1, col + 1 : currTrackLen]), [0; 0]];
            coreDataMatrix.mpTrackMatrix(:, 1:currTrackLen, mpId) = trackCurrMpSet;
        
            coreDataMatrix.mpTracksCountVec(mpId, 1) =  ...
                  coreDataMatrix.mpTracksCountVec(mpId, 1) - 1;
        end
    end    
end