% Predict current frame pose using last frame and before last frame, then
% save it to coreData

function predictPose_m(frameId)
    global params;
    global coreDataMatrix;
    
    if frameId <= 2 %|| frameId > params.imagePaths.Count
        disp('invalid input in PredictPose, please check!');
        return;
    end
%     currIdInner = coreDataMatrix.frImgIdVec(frameId);
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    lastIdInner = coreDataMatrix.frImgIdVec(frameId-1);
    lastlastIdInner = coreDataMatrix.frImgIdVec(frameId-2);

    poseMatrixLast = [coreDataMatrix.frPoseMatrix(:, :, lastIdInner);...
                      0, 0, 0, 1];
    
    poseMatrixLastLast = [coreDataMatrix.frPoseMatrix(:, :, lastlastIdInner);...
                          0, 0, 0, 1];
    
    localPose = poseMatrixLast * pinv(poseMatrixLastLast); 
    curPose = localPose * poseMatrixLast;
    
    coreDataMatrix.frPoseMatrix(:, :, currIdInner) = curPose(1:3,:);
    
end