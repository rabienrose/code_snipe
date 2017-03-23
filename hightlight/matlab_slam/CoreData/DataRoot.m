classdef DataRoot
    
    properties(SetAccess = public, GetAccess = public)
        CurrentFrameId = -1;
    end
    
    properties(SetAccess = public, GetAccess = public)
        imgSize = [-1, -1]; % [width, height]
    end
    
    properties(SetAccess = public, GetAccess = public)
        KFList = []; %todo: rename KFList to FrameList
    end
    
    properties(SetAccess = public, GetAccess = public)
        MPList = [];
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods (Access = public)
        %-init-FrameList-------------------------------------------------
        function this = init(this, varargin)
            frameCount = varargin{1};
            frame = KeyFrameSlam;
            this.KFList = repmat(frame, frameCount, 1);
        end

        %-add-key-frame--------------------------------------------------
        function this = updateKeyFrameSlam(this, varargin)
            frameId = varargin{1};
            keyFrame = varargin{2};
            this.KFList(frameId) = keyFrame;
        end
        
        %-add-map-point--------------------------------------------------
        function this = addMapPointSlam(this, varargin)
           mapPoint = varargin{1};
           
           if isempty(this.MPList)
                this.MPList = mapPoint;
            else
                this.MPList = [this.MPList; mapPoint];
            end
        end
        
        
    end
end