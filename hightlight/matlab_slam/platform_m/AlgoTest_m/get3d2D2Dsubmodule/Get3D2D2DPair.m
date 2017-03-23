%% Generata Sythetic data of 3D-2D points and corresponding camera pose in several frames
% Input:   Dataparams structure
%          Dataparams.cameraparams, {W, H, Theta}
%          Dataparams.datanumber, 3D/2D points number
%          Dataparams.datarange, [x_lim; y_lim; z_lim];
%                     x_lim: range of x coordinate
%                     y_lim: range of y coordinate
%                     z_lim: range of z coordinate
%          Dataparams.datatype;
%                    0, have rotatio and translation;  
%                    1, fix R (no rotation, move along a straight line);
%                    2, fix t (no transalation)
%          Dataparams.randtype, 0 for random choose; 1 for fixed seed

% Output: 
%        points3D: 3D points of each frame
%        points2D: 2D points of each frame
%        flag2D: Indicate 2D points that are in the view port
%        viewpointsnum: the number of points that are in the view port
%        AllR: all rotation matrices(here R is the matrix in the world coordinate)
%        Allt: all translation vectors(here t is the vector in the world coordinate)
%        K: camera intrinsic matrix

function [points3D, points2D, flag2D, viewpointsnum, AllR, Allt, K] = Get3D2D2DPair(Dataparams) 
 %% Set camera Intrinsic Matrix  
   % image width (pixel)  
     W = Dataparams.cameraparams.W;
   % image height (pixel)
     H = Dataparams.cameraparams.H;
   % field of view (degree)
     theta = Dataparams.cameraparams.Theta;
   % get the intrinsic matrix
     K = getIntrinsic(W, H, theta);
     
%% Generate Sythetic 3D points
   % define points number
     pointsNum = Dataparams.datanumber;
   % generate random 3D points
   % randtype: 0 for random choose; 1 for fixed seed
     x_lim = Dataparams.datarange(1, :);
     y_lim = Dataparams.datarange(2, :);
     z_lim = Dataparams.datarange(3, :);
     Points3D = get3dpoints(x_lim, y_lim, z_lim, pointsNum, Dataparams.randtype);

%% Generate  camera translation vectors
   % assume that camera is moving around circumference
   % diameter of circumference
     diam = Dataparams.framedecider.diam; 
   % define the rotation angle each time (degree)
     angle = Dataparams.framedecider.eachangle; 
   % total rotation angle in the translation (degree)
     totalAngle = Dataparams.framedecider.totalAngle;
   % distance from the 1st frame to the last frame  
     length = Dataparams.framedecider.length;
   % every step size along a straight line
     stepsize = Dataparams.framedecider.stepsize;
   % generate all translation vectors ( mention that first vector is the original: [diam/2, 0, 0])
     datatype = Dataparams.datatype;
     allt = getTransVectors(diam, angle, totalAngle, length, stepsize, datatype);

   
     %plot(allt(:, 1), allt(:, 2), '*');
 
%% Generate  camera Rotation Matrices  
   switch datatype
       case 0  
              % compute the ratation around a circle
                for i = 1 : size(allt, 1)-1
                    delVec = allt(i+1, :)-allt(i, :);
                    delVec = delVec/norm(delVec);
                    dirz = delVec';
                    diry = [0; 1; 0];
                    dirx = cross(diry, dirz);
                    allR{i} = [dirx diry dirz];
                end
        
       case 1   % fix R
                        % original (no ratation)
                for i = 1 : size(allt, 1)
                    allR{i} = [1, 0, 0;
                               0, 1, 0;
                               0, 0, 1]; 
                end
           
       case 2   % fix t
                type = 0;
                allt1 =  getTransVectors(diam, angle, totalAngle, length, stepsize, type);
             % compute the ratation around a circle
                for i = 1 : size(allt1, 1) - 1
                    delVec = allt1(i+1, :) - allt1(i, :);
                    delVec = delVec/norm(delVec);
                    dirz = delVec';
                    diry = [0; 1; 0];
                    dirx = cross(diry, dirz);
                    allR{i} = [dirx diry dirz];
                end
        
       otherwise
               disp('Unexpected type!')    
   end

%% Project 3D points to the 2D plane
   % construt Projection matrices (## It should be mentioned that in matlab:
   % [x, y, 1] = [X, Y, Z, 1] * camMatrix;  camMatrix = [R'; t] * K##;
     for indPro = 1 : size(allR, 2)
         homoTrans = [allR{indPro} allt(indPro, :)'; 0 0 0 1]^-1;
         WorldR{indPro} = homoTrans(1 : 3, 1 : 3);
         Worldt(indPro,:) = homoTrans(1 : 3, 4)';
         P{indPro} = K * homoTrans(1:3, :);
     end
   % Project 3D points to the 2D plane 
     % reconstrut 3D points with homogeneous coordinates 
     points3DHomo = [Points3D, ones(size(Points3D, 1), 1)];
     
     % projection  % meanwhile transform 2D points to the inhomogeneous coordiantes
     for indPosition = 1 : size(P, 2) 
         % initialized  the counter
         pointsnumInhomo = 0;
         % initialize the flag of points that are located at the view port
         flagpoints2D = zeros(size(Points3D, 1), 1);
         
         % compute the 2D points in Homogeneous/InHomogeneous case
         for indInHomo = 1 : size(points3DHomo, 1)
            %  Projection by Matrix P
                points2DHomo(indInHomo, :) = P{indPosition} * points3DHomo(indInHomo, :)';
                current2D = points2DHomo(indInHomo, :);    
            %   check if the 2D point in the back of the camera   
                points2DInHomo(indInHomo, :) = current2D./current2D(3); 
                if current2D(3) <0
                    continue;
                end             
            %  check if the current 2D point is in the view port (W * H pixels) 
            current2DInHomo = points2DInHomo(indInHomo, :);
            if current2DInHomo(1) <= W  &&  current2DInHomo(2) <= H && current2DInHomo(1) > 0  &&  current2DInHomo(2) > 0 ...
%                 && Points3D(indInHomo, 3) > 6  &&  Points3D(indInHomo, 3) <= 10  
                pointsnumInhomo = pointsnumInhomo + 1;
                flagpoints2D(indInHomo) = 1; 
            end
         end 
         % count the number of points that are in the view port
         viewpointsnum(indPosition, 1) = pointsnumInhomo;
         flagpoints2Dtotal{indPosition} = flagpoints2D;
         % collect 2D points 
         points2DcolletInHomo{indPosition} = points2DInHomo;
     end
     
   % get final output data 
     points3D = Points3D;
     points2D = points2DcolletInHomo; 
     flag2D = flagpoints2Dtotal;
     AllR = WorldR;
     Allt = Worldt;
end




 
