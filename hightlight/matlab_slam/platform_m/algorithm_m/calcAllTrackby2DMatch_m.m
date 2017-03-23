% Calculate all 2D-2D tracks using Optical flow.

function [isSucc] = calcAllTrackby2DMatch_m(imageIdxSet)
    global params;
    global coreDataMatrix;
    
    if(imageIdxSet(end) > params.imagePaths.Count)
        isSucc = false;
        return;
    end
    
    idxInner = coreDataMatrix.frImgIdVec(imageIdxSet);
    fr1stInner = idxInner(1);
    lastInner = idxInner(end);
    for i = fr1stInner + 1 : lastInner
        matchByOptiFlowCore_m(i, -1);
    end
    isSucc= true;
end



