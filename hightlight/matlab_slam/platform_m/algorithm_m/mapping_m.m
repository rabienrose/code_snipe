% Tracking module
% Author: Zhongjun Dai
% Date: 2016-7-7

function [isSucc] = mapping_m(frameId)
    for j=1 : 10
        matchByEpiCore_m(frameId, -j);
        calcMPbyTrack_m(frameId, -j);
    end
%     if frameId >= 5
%         frameCount = 10;
%         if mod(frameId, 5) == 0
%             [isSucc, ~] = BAbyCoreData2(frameId, frameCount);
%         end
%     end
    isSucc = true;
end
