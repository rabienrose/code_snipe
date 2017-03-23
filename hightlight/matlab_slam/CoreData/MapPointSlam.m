classdef MapPointSlam
      
    properties(SetAccess = public, GetAccess = public)
        Posi = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        Track = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        Descriptor = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        isBad = false;
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods (Access = public)
        %-get the index of kp in specific frame in track.
        function kpInd = getMatchedPt(this, varargin)
           kpInd = -1;
           tarFrameId = varargin{1};
           for i=1:size(this.Track,1)
                if this.Track(i).FrameId == tarFrameId 
                    kpInd = this.Track(i).KPId;
                    return;
                end
           end
        end
        
        %-del trackItem in track.
        function [kpInd, this] = delTrackItem(this, varargin)
           kpInd = -1;
           tarFrameId = varargin{1};
           for i=1:size(this.Track, 1)
                if this.Track(i).FrameId == tarFrameId 
                    kpInd = this.Track(i).KPId;
                    this.Track(i,:) = [];
                    return;
                end
           end
        end
    end
end