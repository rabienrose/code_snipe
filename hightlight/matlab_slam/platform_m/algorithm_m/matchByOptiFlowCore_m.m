% Calculate 2D-2D tracks of 2 images using Optical flow.

function [isSucc] = matchByOptiFlowCore_m(frameId, offset)  
    global params;
    global coreDataMatrix;
    
    if frameId + offset < 1 || frameId > params.imagePaths.Count
        disp(['Absolute frame', num2str(frameId), ': invalid input in matchByOptiFlowCore, please check!']);
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    prevFrameId = currIdInner + offset;
    if prevFrameId < 1 || currIdInner > size(coreDataMatrix.kpsCountVec, 1)
        disp(['Inner frame', num2str(currIdInner), ': invalid inner input in matchByOptiFlowCore, please check!']);
        return;
    end
    
    framePrev = imread(coreDataMatrix.frImgDirMatrix{prevFrameId});
    frameCurr = imread(coreDataMatrix.frImgDirMatrix{currIdInner});
    
    stKeyPoint = struct('Location', [],...
                        'Octave', [],...
                        'Scale', [],...
                        'Metric', []...
                       ); 
    
    col = coreDataMatrix.kpsCountVec(prevFrameId);       
    ptsPrev = stKeyPoint;
    ptsPrev.Location = cast(coreDataMatrix.kpPosiMatrix(:, 1 : col, prevFrameId)', 'single');
    ptsPrev.Octave = cast(coreDataMatrix.kpOctaveMatrix(1 : col, prevFrameId), 'uint32');
    ptsPrev.Scale = cast(zeros(col, 1), 'single');
    ptsPrev.Metric = cast(zeros(col, 1), 'single');
    descriptorPrev = coreDataMatrix.kpDescMatrix(:, 1 : col, prevFrameId);
    
    col = coreDataMatrix.kpsCountVec(currIdInner, 1);
    ptsCurr = stKeyPoint;
    ptsCurr.Location = cast(coreDataMatrix.kpPosiMatrix(:, 1 : col, currIdInner)', 'single');
    ptsCurr.Octave = cast(coreDataMatrix.kpOctaveMatrix(1 : col, currIdInner), 'uint32');
    ptsCurr.Scale = cast(zeros(col, 1), 'single');
    ptsCurr.Metric = cast(zeros(col, 1), 'single');
    descriptorCurr = coreDataMatrix.kpDescMatrix(:, 1 : col, currIdInner);

    [matches pose] = OptiFlow_m(framePrev, frameCurr, ptsPrev, ptsCurr, descriptorPrev', descriptorCurr', params.cameraParam);
    matches = matches + 1;
    fill2DMatches_m(prevFrameId, currIdInner, matches);
    isSucc = true;
end



