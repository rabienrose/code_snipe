classdef KeyPointSlam
    properties(SetAccess = public, GetAccess = public)
        posi = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        descriptor = [];
    end
    
    properties(SetAccess = public, GetAccess = public)
        octave = -1;
    end
    
    properties(SetAccess = public, GetAccess = public)
        mpId = -1;
    end
end