%%Tracker definition
%Created by zhaiq 2016.6.24
classdef Tracker
    properties(SetAccess = public, GetAccess = public)        
        m_pose            = [];
        m_points3D        = [];
        m_points3DDesc    = [];
        m_points2D        = [];
        m_bvalid          = [];        
    end

    properties(SetAccess = public, GetAccess = public)        
        % frame info
        m_currFrame = [];   %type of Frame, all frame information is saved in Frame.
        
        % tracker info
        m_motion    = [];
        m_lastFrame = [];
        
        %write to file
        m_fileId = []
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods (Access = public)
        %% init tracker        
        function this = init(this, varargin)
            %@param varargin{1} Parameters file path
            
            % init current frame
            this.m_currFrame = Frame;
            this.m_currFrame = this.m_currFrame.init(varargin{1});
            
            this.m_fileId = fopen('test_search_by_projection.txt', 'w');
        end
                
        %% do tracking       
        function [this, pose, points3D, points3DDesc, points2D, bvalid] = track(this, varargin)
            %pass current frame, all map points(points3D) to tracker, return
            %optimized current camera pose, updated map points to caller.
            %@param varargin{1, 1} Current object
            %@param varargin{1, 2} Current frame
            %@param varargin{1, 3} Last tracked pose
            %@param varargin{1, 4} All map points need to track
            %@param varargin{1, 5} Descriptors of all map points.
            %@param this           [output]
            %@param pose           [output]
            %@param points3D       [output]
            %@param points3DDesc   [output]
            %@param points2D       [output]
            %@param bvalid         [output]
        
            %detect key and calcualte descriptors using current frame.
            this.m_currFrame = this.m_currFrame.updateFrameInfo(varargin{1, 2});
            
            %track
            bvalid = [];
            [this, bvalid] = this.trackWithMotion(varargin{1, 3}, varargin{1, 4}, varargin{1, 5}, bvalid);
            
            state = 1;
            if(state)
                state = this.trackLocalMap();
            end
            
            %after tracking, update tracker info
            if(state)
                this = this.updateTrackerInfo();
            end
            
            %output, optimized camera pose, map points, descriptors and 
            %outliers identifier to caller.
        end
    end
    
    methods (Access = private)
        
        function [this, bvalid] = trackWithMotion(this, varargin)
            %@param varargin{1, 1} Last tracked pose.
            %@param varargin{1, 2} All mappoints need to track.
            %@param varargin{1, 3} Descriptors of all map points.
            %@param varargin{1, 4} Outliers flag of all map points.
            %@param this   [output]Current object.
            %@param bvalid [output]Validity identifier. 
            
            %initialize current camera pose using last tracked pose and
            %motion model.
            if(isempty(this.m_motion))
                this.m_currFrame = this.m_currFrame.setPose(varargin{1});
            else
                this.m_currFrame = this.m_currFrame.setPose(this.m_motion * this.m_lastFrame.m_pose3D);
            end
            
            %get 3D-2D matching pairs for pose estimation.
            [this, bvalid] = this.searchByProjection(this.m_currFrame, varargin{1}, varargin{2}, varargin{3});
            
            %call optimizer using mex.
            %[this.m_pose, bflag] = optimizePose(camera_param, initial_pose, points3D_x, points3D_y, points3D_z, points2D_x, points2D_y, octaves);
        end
        
        function [this, valid] = searchByProjection(this, varargin)
            %@param varargin{1, 1} Current frame info
            %@param varargin{1, 2} Last tracked pose
            %@param varargin{1, 3} All mappoints in last frame
            %@param varargin{1, 4} Descriptors of all map points
            %@param varargin{1, 5} Outliers flag of all map points
            %@param [output] Current object
            %@param [output] Validity identifier
        
            %for each key,
            %   project map points into 2D plane using camera pose and camera parameters.
            %   if the postion is out of boundary, discard it, and set
            %   current outlier identifier as true.
            R = this.m_currFrame.m_pose3D(:, 1:3);
            t = this.m_currFrame.m_pose3D(:, 4);
            x3Dw = varargin{1, 3};
            x3Dc = bsxfun(@plus,R * x3Dw',t);
            
            xc = x3Dc(1, :);
            yc = x3Dc(2, :);
            zc_inv = 1.0 ./ x3Dc(3, :);
            
            u = this.m_currFrame.m_cameraParam(1, 1) .* xc .* zc_inv + this.m_currFrame.m_cameraParam(1, 3);
            v = this.m_currFrame.m_cameraParam(2, 2) .* yc .* zc_inv + this.m_currFrame.m_cameraParam(2, 3);
            
%             %%save camera param, pose3d, map points and point in frame to file
%             % camera param
%             for j=1 : 3
%                 for i=1 : 3
%                     fprintf(this.m_fileId, '%f ', this.m_currFrame.m_cameraParam(j, i));
%                 end
%                 fprintf(this.m_fileId, '\n');
%             end
%             
%             % camera pose
%             for j=1 : 3
%                 for i=1 : 4
%                     fprintf(this.m_fileId, '%f ', this.m_currFrame.m_pose3D(j, i));
%                 end
%                 fprintf(this.m_fileId, '\n');
%             end
%             
%             % map points
%             x3Dw = x3Dw';
% 
%             for j=1 : 3
%                 for i=1 : size(x3Dw, 2)
%                     fprintf(this.m_fileId, '%f ', x3Dw(j, i));
%                 end
%                 fprintf(this.m_fileId, '\n');
%             end
%             
%             % points 2d
%             n_u = u';
%             for i=1 : size(n_u)
%                 fprintf(this.m_fileId, '%f ', n_u(i));
%             end
%             fprintf(this.m_fileId, '\n');
%             
%             n_v = v';
%             for i=1 : size(n_v)
%                 fprintf(this.m_fileId, '%f ', n_v(i));
%             end
%             fprintf(this.m_fileId, '\n');
            %%
            
            valid = [];
            for keyId = 1 : size(varargin{1, 3})
                valid(keyId) = 1;
                %if ~varargin{1, 5}(keyId)
                    if zc_inv(keyId) < 0
                        valid(keyId) = 0;
                        continue;
                    end
                    if u(keyId) < this.m_currFrame.m_minX || u(keyId) > this.m_currFrame.m_maxX || ...
                       v(keyId) < this.m_currFrame.m_minY || v(keyId) > this.m_currFrame.m_maxY
                        valid(keyId) = 0;
                        continue;
                    end
                    
                    radius = 11;
                    indices = this.m_currFrame.getFeatureIndexInArea(u(keyId), v(keyId), radius);
                    if(isempty(indices))
                        valid(keyId) = 0;
                        continue;
                    end
                    
                    bestDist = 1e4;
                    bestId = -1;
                    ref_desc = varargin{1, 4}(keyId, :);
                    for id = 1 : size(indices)
%                         if(frameInfo.m_valid(indices(id)))
%                             continue;
%                         end
                        
                        desc = this.m_currFrame.m_descriptors(indices(id), :); % multi dimension                        
                        dist = this.computeDescriptorBestDistance(ref_desc, desc); % for one key points, may be multiple descriptors extracted.
                        if(dist < bestDist)
                            bestDist = dist;
                            bestId   = indices(id);
                        end 
                    end
                    if(bestDist < 100)
                        this.m_currFrame.m_points3D(bestId) = varargin{1, 3}(keyId);
                        this.m_currFrame.m_outliers(bestId) = 0;
                    end
                %end
            end
        end
        
        function this = updateTrackerInfo(this)
            %
            this.m_lastFrame = this.m_currFrame;
        end
        
        %% compute
        function bestDist = computeDescriptorBestDistance(~, varargin)
            %@param varargin{1, 1} Reference descriptor
            %@param varargin{1, 2} Descriptors set

            bestDist = 1e4;
            for descId = 1 : size(varargin{1, 2})
                dist = computeDistance(varargin{1, 1}, varargin{1, 2}(descId, :));
                if dist < bestDist
                    bestDist = dist;
                end
                
%                 %
%                 desc1 = varargin{1, 1};
%                 desc2 = varargin{1, 2}(descId, :);
%                 
%                 for id = 1 : size(desc1, 2)
%                     fprintf(this.m_fileId, '%12.8f ', desc1(1, id));
%                 end
%                 fprintf(this.m_fileId, '\n');
%                 
%                 for id = 1 : size(desc2, 2)
%                     fprintf(this.m_fileId, '%12.8f ', desc2(1, id));
%                 end
%                 fprintf(this.m_fileId, '\n');
%                 
%                 fprintf(this.m_fileId, '%6.4f\n', dist);
%                 %
            end
            
            function answer = computeDistance(desc1, desc2)
                %assert(size(desc1, 2) == size(desc2, 2));
                d = 0;
                for idx = 1 : size(desc1, 2)
                    d = d + (desc1(1,idx) - desc2(1,idx)) * (desc1(1,idx) - desc2(1,idx));
                end
                answer = sqrt(d);
            end
        end
        
        function this = trackLocalMap(this, varargin)
            % update local keypoints and local map points.
            this.updateLocalMap(varargin); % todo: manage local map in caller?
            
            % project map points in local map to current frame using camera
            % pose estimated just now, update the matching pairs.
            this.searchLocalPoints(varargin);
            
            % call optimizer from mex, optimize camera pose using updated 
            % matching pairs.
            optimizePose();
            
        end
        
        function this = updateLocalMap(this, varargin)
            
        end
        
        %% 
        function this = searchLocalPoints(this, varargin)
            % Do not search map points already matched.
            
            
            % Project points into frame and check its visibility, check
            % whether projected points in in the visibility frustum.
            
            % Search matches by projected result.

        end
    end
    
end