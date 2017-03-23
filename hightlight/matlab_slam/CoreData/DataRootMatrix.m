classdef DataRootMatrix   
    
    properties(SetAccess = public, GetAccess = public)
        imgSize = [-1, -1]; % [width, height]
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpTrackMatrix = []; 
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpIsBadVec = []; 
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpDescMatrix = []; 
    end
        
    properties(SetAccess = public, GetAccess = public)
        mpPosiMatrix = []; 
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpCount = -1; 
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpTracksCountVec = []; 
    end
    
%% Frames

    properties(SetAccess = public, GetAccess = public)
        frPoseMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        frTypeMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        frImgDirMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        frImgIdVec = [];
    end
    
 %%  kyePoints
 
    properties(SetAccess = public, GetAccess = public)
        kpPosiMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        kpDescMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        kpMpIdMatrix = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        kpOctaveMatrix = [];
    end

    properties(SetAccess = public, GetAccess = public)
        kpsCountVec = [];
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
%     methods (Access = public)
%        %-get the index of kp in specific frame in track.
%         function kpInd = getMatchedPt(currTrack, tarFrameId)
%            [~, col] = find(currTrack(1, :) == tarFrameId);
%            if isempty(col)
%                warning('This FrameId is not in the track!');
%                kpInd = -1;
%            else
%                kpInd = currTrack(2, col);
%            end                     
%         end 
        
       %-del trackItem in track. (here just consider delete a single [frameId, kpId])
%         function [kpInd, this1, this2] = delTrackItem(this1, this2, tarFrameId, mpId)
%            kpInd = -1;
%            currTrackLen = this2(mpId, 1);
%            for i = 1 : currTrackLen;
%                 if this1(1, i) == tarFrameId 
%                     kpInd = this1(2, i);
%                     % del the currtrackItem (i)
%                     this1_copy = [this1(:, [1 : i - 1, i + 1 : currTrackLen]), [0; 0]];
%                     this1 = this1_copy;
%                     % redefine the mpTracksCountVec
%                     this2(mpId, 1) = currtrackLen - 1;
%                     return;
%                 end
%            end
%         end
%        %---------------------------------------------------------------
%     end
    %----------------------------------------------------------------------
end




