% Calculate all features using assigned method in config.

function [isSucc] = calcAllFeatures_m(imageIdxSet)

    global params;
    global coreDataMatrix;
    
    startFrame = imageIdxSet(1, 1);
    frameCount = size(imageIdxSet, 1);
    
    % updata frCount
    coreDataMatrix.frCount = frameCount;
    
    if(imageIdxSet(frameCount, 1) > params.imagePaths.Count)
        isSucc = false;
        return;
    end
    
    % fr1stInner is different from the start Frame
    fr1stInner = coreDataMatrix.frImgIdVec(startFrame, 1); 
    currFrame = 1;
    for i = fr1stInner : fr1stInner + frameCount - 1
        frameId = imageIdxSet(i, 1);
        calcFeaturesOneFrame_m(frameId, currFrame);
        currFrame = currFrame + 1;
    end 
    
end

  