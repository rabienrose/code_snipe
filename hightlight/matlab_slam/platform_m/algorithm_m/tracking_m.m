% Tracking module

function [isSucc] = tracking_m(frameId)
    %detect keys and extract descriptors in current frame.
    % do this work in batch in initilisation
%     if ~calFeaturesOneFrame_m(frameId)
%         disp('Tracking Err: failed to calculate features.');
%         isSucc = false;
%         return;
%     end

    predictPose_m(frameId);
    
    %compute a coarse pose using current frame and previous frame matching
    %info.
    if ~matchByProjectionCore_m(frameId, 1)        
        isSucc = false;
        disp('Tracking Err: failed to search matches.');
        return;
    end
    
    if ~poseOptimization_m(frameId)
        isSucc = false;
        disp('Tracking Err: failed to optimize pose.');
        return;
    end
    
    %refine pose using more matching pairs.
    if ~matchByProjectionCore_m(frameId, 20)
        isSucc = false;
        disp('Tracking Err: failed to search matches.');
        return;
    end
    
    if ~poseOptimization_m(frameId)
        isSucc = false;
        disp('Tracking Err: failed to optimize pose.');
        return;
    end
    
    isSucc = true;
end

