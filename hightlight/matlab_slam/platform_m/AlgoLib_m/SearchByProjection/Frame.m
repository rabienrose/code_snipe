%%definition for Frame, save all information of frame.
classdef Frame
    properties(SetAccess = public, GetAccess = public)
        %frame info
        m_points2D    = []; %all feature points in current frame.
        m_descriptors = []; %all descriptors of feature points, if 
                            %multi-descriptors are calculated for a single 
                            %point, eg, SIFT, then m_descriptors is a K x N
                            %matrix, K is the descriptor number,N is the 
                            %dimesion of feature, in general, K=1.
        
        m_points3D    = []; %all map points related to feature points.
        m_outliers    = []; %feature points outlier indentifier
        m_valid       = [];

        m_minX        = [];
        m_maxX        = [];
        m_minY        = [];
        m_maxY        = [];
        
        %camera info
        m_cameraParam = []; % camera intrinsic parameters, 3 x 3 float matrix.
        m_distCoef    = []; % camera distortion parameters
        m_pose3D      = []; % camera external parameters, 3 x 4 float matrix.
       
        %feature detector and extractor
        m_keyType           = [];
        m_descType          = [];
    end
    
    methods (Access = public)
        %% init frame
        %@param varargin{1} Configuration file path.
        function this = init(this, varargin)
            addpath('../MainFlow');
            global params;
            config;
            %step1.save camera info
            %[this.m_cameraParam, this.m_distCoef] = loadCameraParamFromFile(varargin{1});
            this.m_cameraParam = params.cameraParam;
            this.m_distCoef = params.distCoef;
            
            %step2.save keys type and descriptors type
            %[this.m_keyType, this.m_descTye] = loadFeatureInfoFromFile(varargin{1});
            this.m_keyType = params.featureType;
            this.m_descType = params.descType;
        end
        
        %% update frmae info using input frame.
        function this = updateFrameInfo(this, varargin)
            %@param varargin{1, 1} Frame
        
            %step1.detect keys on input frame based on key type.
            I = varargin{1, 1};
            points = DetectKeyPoints(I, this.m_keyType);
            this.m_points2D = points.Location;
            
            %step2.calculate descriptors using detected keys, based on
            %descriptor type
            this.m_descriptors = ExtractFeatures(I, points, this.m_keyType);
            
            %step3.initialize points3D and outliers identifier array.


            this.m_minX = 0;
            this.m_minY = 0;
            this.m_maxX = size(varargin{1, 1}, 2);
            this.m_maxY = size(varargin{1, 1}, 1);
            
            %add by zhaiq:
            %NOTE!!!: the strategy split keys into grids, which is based on
            %ORB-SLAM, would be used in future.
        end
        
        %% update current camera pose using the given one.
        %@param varargin{1} Camera pose which is set to this.
        function this = setPose(this, varargin)
            this.m_pose3D = varargin{1};
        end
        %% get frame pose (todo)
        function pose = getPose(this)
            pose = this.m_pose3D;
        end
        
        %% read current frame info(todo)
        function [points2D, descriptors, points3D, outliers, pose3D, cameraParam] = getFrameInfo(this)
            points2D    = this.m_points2D;
            descriptors = this.m_descriptors;
            points3D    = this.m_points3D;
            outliers    = this.m_outliers;
            pose3D      = this.m_pose3D;
            cameraParam = this.m_cameraParam;
        end
        
		function indices = getFeatureIndexInArea(this, varargin)
            %get keys' index according to given area
            %@param varargin{1, 1} x-axis coordinate matrix
            %@param varargin{1, 2} y-axis coordinate matrix
            %@param varargin{1, 3} raduis
            
            %assert(size(varargin{1, 1}) == size(varargin{1, 2}));
            area = [bsxfun(@minus,varargin{1, 1}(1), varargin{1, 3}(1));...
                    bsxfun(@minus,varargin{1, 2}(1), varargin{1, 3}(1));...
                    bsxfun(@plus,varargin{1, 1}(1), varargin{1, 3}(1));...
                    bsxfun(@plus,varargin{1, 2}(1), varargin{1, 3}(1))];
            area = area';
               
            x = this.m_points2D(:, 1);
            y = this.m_points2D(:, 2);

            for num = 1 : size(area, 1)
                indices = find(x >= area(num, 1) & x <= area(num, 3) & y >= area(num, 2) &  y<= area(num, 4));
            end
        end
    end
end