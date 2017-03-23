% Config the the information of generating Synthetic data parameters

global Dataparams;

% Get the directory of image to generate fake descriptor.
% Dataparams.fakeimageDir = '../images';


%% ---------------------------------------------------------------------------------------
%                                Set camera parameters  
%-----------------------------------------------------------------------------------------
% image width (pixel)  
  Dataparams.cameraparams.W = 1241;
  
% image height (pixel)
  Dataparams.cameraparams.H = 376;
  
% field of view (degree)
  Dataparams.cameraparams.Theta = 90;

%% ---------------------------------------------------------------------------------------
%                          define parameters of synthetic data  
%-----------------------------------------------------------------------------------------
% define the range
  min_range = -10;
  max_range =  10;
  x_lim = [min_range, max_range];
  y_lim = [min_range, max_range];
  z_lim = [min_range, max_range];  
  Dataparams.datarange = [x_lim; y_lim; z_lim];
  
% define the number of points
  Dataparams.datanumber = 2000;
  
% define Datatype: 0, have rotation and translation; 1, fix R (no rotation, move along 
% a straight line); 2, fix t (no transalation)
  Dataparams.datatype = 1;
  
% define the parameters that control the total frames of synthetic data
  % for datatype 0: define the diameter of circle; move around a circle and 
  % each move angle is eqaul to the ratation angle in the datatype 2
  Dataparams.framedecider.diam = 14;
  % for datatype 1: define distance from the 1st frame to the last frame; 
  % and step size along a straight line (frame Number = length / stepsize)
  Dataparams.framedecider.length = 12;
  Dataparams.framedecider.stepsize = 3;
  % for datatype 2: define the each rotation angle total rotation angle(degree)
  % (frame Number = totalAngle / eachangle)
  Dataparams.framedecider.totalAngle = 360;
  Dataparams.framedecider.eachangle = 12;
  
  
% randtype: 0 for random choose; 1 for fixed seed( change seed number in get3dpoints.m)
  Dataparams.randtype = 0;   

% using some images for background 
  Dataparams.imageDir = '/Users/test/Desktop/Videos/image_0/';
  Dataparams.imagePaths = imageSet(Dataparams.imageDir);
%% ---------------------------------------------------------------------------------------
%                        define some options to fill the coreData  
%-----------------------------------------------------------------------------------------
% if the showtrajectory == 1, show the camera trajectory   
  Dataparams.showtrajectory = 0;
  
% show the track of FrameId and previous FrameId (FrameId >= 2)
  Dataparams.ShowFrameId = 4;

% set the currframeId of coredata
  Dataparams.currframeId = 2;
  
% Fill the date frame up to Frame (current max frame number is totalAngle / eachangle)
%   if Dataparams.datatype == 1
%         Dataparams.filluptoframe = Dataparams.framedecider.length / Dataparams.framedecider.stepsize;
%   else
%         Dataparams.filluptoframe = Dataparams.framedecider.totalAngle / Dataparams.framedecider.eachangle;  
%   end
  
% fill the 3D positions('1') or not('0') 
  Dataparams.fill3dposi = 1;
  

%% ---------------------------------------------------------------------------------------
%                        add noise to the pure coreData  
%-----------------------------------------------------------------------------------------
% add the noise to the 3D positions ('1') or not('0') 
  Dataparams.addnoise3dposi = 0;
  
% add the noise to the 2D positions ('1') or not('0') 
  Dataparams.addnoise2dposi = 0;
  
% add the noise to the fake descriptor ('1') or not('0') 
  Dataparams.addnoisedescriptor = 0;  

% add the noise to the Pose(R and t) ('1') or not('0') (here R should maintain the property of 
% being a roration matrix )
  Dataparams.addnoise2poseR = 0;
  Dataparams.addnoise2poset = 0;

%----------------------------------------------------------------------------------------

% noise distribution type('Normal', 'Uniform', 'Gamma')
  Dataparams.noisetype = 'Normal';
%   Dataparams.noisetype = 'Uniform';
%   Dataparams.noisetype = 'Gamma';
  
% mean and standard deviation of the normal distribution
  Dataparams.Normalnoise.mu = 0;
  Dataparams.Normalnoise.std = 0.2;

% a: lower endpoint (minimal) and b: upper endpoint (maximal) of the uniform distribution
  Dataparams.Uniformnoise.a = -0.02;
  Dataparams.Uniformnoise.b = 0.02;
 
% a: shape parameter and the b: scale parameter of the gamma distribution  
  Dataparams.Gammanoise.a = 0.1;
  Dataparams.Gammanoise.b = 0.1;
     
% add noise to the 2D points of specific frames {1,..., 30}
  Dataparams.addnoise2dframesId = [1, 2, 3];  
  
% add a small drift angle (degree) in the direction of {x-axis, y-axis ,z-axis} as noise to the Pose 
  Dataparams.addposedriftangle = [30, 0, 0];
  
% add noise to the Pose of specific frames {1,..., 30}
  Dataparams.addnoisePoseRframesId = [1, 2, 3, 5];
  Dataparams.addnoisePosetframesId = [1, 2, 3, 5];
  
  



