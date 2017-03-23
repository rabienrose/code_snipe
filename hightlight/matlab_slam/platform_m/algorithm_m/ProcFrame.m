% This function applys a complete procedure to do a single frame slam
% Author: Zhongjun Dai
% Date: 2016-7-1

function [isSucc] = ProcFrame(frameId)

    %tracking
    status = Tracking(frameId);
    if (~status)
        isSucc = false;
        return;
    end
    
    %mapping
    status = Mapping(frameId);
    if (~status)
        isSucc = false;
        return;
    end
    
    isSucc= true;
end