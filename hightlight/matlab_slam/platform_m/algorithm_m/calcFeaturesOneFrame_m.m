
function [isSucc] = calcFeaturesOneFrame_m(frameId, frameCount)

    global params;
    global coreDataMatrix;
    
    % check boundary
    if frameId < 1 || frameId > params.imagePaths.Count
        isSucc = false;
        disp('invalid input in CalFeaturesOneFrame, please check!');
        return;
    end
    
    I = read(params.imagePaths, frameId);
    if size(I, 3) > 1
        I = rgb2gray(I);
    end
    
    % fill the coreDataMatrix of
    [points1 , desc1] = featureExtractor_m(I, params.featureType, params.descType);
    kpCount = size(points1.Location, 1);
    % only keep 2000 key points
    if kpCount > 2000
        kpCount = 2000;
    end
    frameIdInner = coreDataMatrix.frImgIdVec(frameId);
    coreDataMatrix.kpPosiMatrix(:, 1:kpCount, frameIdInner) = points1.Location(1:kpCount, :)';
    coreDataMatrix.kpDescMatrix(:, 1:kpCount, frameIdInner) = desc1(1:kpCount, :)';
    coreDataMatrix.kpOctaveMatrix(1:kpCount, frameIdInner) = points1.Octave(1:kpCount, 1);
    coreDataMatrix.kpsCountVec(frameCount, 1) = kpCount;
    
    isSucc = true;
    
end

