%% get the camera intrinsic matrix
% here we use the simple model:
%  i.e.  K = [  f  0  cx;
%               0  f  cy;
%               0  0   1]
% input: 
%        W: image width;
%        H: image height;
%        theta: field of view
% output: 
%        K: camera intrinsic matrix

function K = getIntrinsic(W,H,theta)   
    
     % get the focal length of camera, since: tan(theta/2) = W/(2f)
       theta1 = theta/2;     
       f = W/2*inv(tan(theta1 * pi /180));
     
     % define the cx and cy
       cx = W/2;
       cy = H/2;
     % define the intrinsic matrix
       K  = [f, 0, cx;
             0, f, cy;
             0, 0,  1]; 
end