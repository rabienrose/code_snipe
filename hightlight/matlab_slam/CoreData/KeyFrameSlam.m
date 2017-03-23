classdef KeyFrameSlam
    
    properties(SetAccess = public, GetAccess = public)
        Image = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        Pose = table();
    end
    
    properties(SetAccess = public, GetAccess = public)
        KPList = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        status = -1;
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods (Access = public)
        
    end
end