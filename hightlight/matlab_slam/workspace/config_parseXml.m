% Load configuration information from xml file and parse them to matlab.
global params;
addpath('../config');
paramData = xml2struct(configAddr);
params.imageDir = imageDir;

% --------------------------------------------------------------------------------------------
%  PARSE camera parameters
% --------------------------------------------------------------------------------------------
fx = str2double(paramData.opencv_storage.CameraParam.fx.Text);
fy = str2double(paramData.opencv_storage.CameraParam.fy.Text);
cx = str2double(paramData.opencv_storage.CameraParam.cx.Text);
cy = str2double(paramData.opencv_storage.CameraParam.cy.Text);

k1 = str2double(paramData.opencv_storage.CameraParam.k1.Text);
k2 = str2double(paramData.opencv_storage.CameraParam.k2.Text);
p1 = str2double(paramData.opencv_storage.CameraParam.p1.Text);
p2 = str2double(paramData.opencv_storage.CameraParam.p2.Text);

% --------------------------------------------------------------------------------------------
%  PARSE key and descriptor type
% --------------------------------------------------------------------------------------------
params.featureType = paramData.opencv_storage.KeyType.Text;
params.descType = paramData.opencv_storage.DescriptorType.Text;

% --------------------------------------------------------------------------------------------
%  static matrix parameters
% --------------------------------------------------------------------------------------------
params.kpNum = 2000;
params.mpIdLength = 11;

params.cameraParam = [ fx      0      cx; 
                       0       fy     cy; 
                       0       0      1 ];

params.distCoef = [k1, k2, p1, p2];

% --------------------------------------------------------------------------------------------
%  RaulORB Parameters
% --------------------------------------------------------------------------------------------

% number of features per image
params.RaulORB_nDsecDim =  0; % define here if known or update in the initialization

% PARSE number of features per image
params.RaulORB_nFeatures = str2double(paramData.opencv_storage.RaulORBParam.raulFeatureNum.Text);

% PARSE scale factor between levels in the scale pyramid 	
params.RaulORB_scaleFactor = str2double(paramData.opencv_storage.RaulORBParam.raulScaleFactor.Text);

% PARSE number of levels in the scale pyramid	
params.RaulORB_nLevels =  str2double(paramData.opencv_storage.RaulORBParam.raulLevelNum.Text);

% PARSE Fast threshold
% Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
% Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
% You can lower these values if your images have low contrast		
params.RaulORB_iniThFAST = str2double(paramData.opencv_storage.RaulORBParam.FASTInitialTh.Text);
params.RaulORB_minThFAST = str2double(paramData.opencv_storage.RaulORBParam.FASTMinTh.Text);
params.RaulORB_initFlag  = 0;

% calculate 4 kind of scale sets
scaleFactorSet = zeros(1, params.RaulORB_nLevels);
scaleFactorSet(1, 1) = 1.0;
scaleFactorSigma2 = zeros(1, params.RaulORB_nLevels);
scaleFactorSigma2(1, 1) = 1.0;
for l = 2 : params.RaulORB_nLevels
    scaleFactorSet(1, l) = scaleFactorSet(1, l - 1) * params.RaulORB_scaleFactor;
    scaleFactorSigma2(1, l) = scaleFactorSet(1, l)^2;
end
inv_scaleFactorSet = 1.0 ./ scaleFactorSet;
inv_scaleFactorSigma2 = 1.0 ./ scaleFactorSigma2;

params.scaleFactorSet = scaleFactorSet;
params.scaleFactorSigma2 = scaleFactorSigma2;
params.inv_scaleFactorSet = inv_scaleFactorSet;
params.inv_scaleFactorSigma2 = inv_scaleFactorSigma2;

%%
pInstance   = cast(0, 'int64');

