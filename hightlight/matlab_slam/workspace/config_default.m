% Load camera information and freature information
% Author: Zhongjun Dai
% Date: 2016-6-26

global params;
params.imageDir = imageDir;
k1 = 0.0;
k2 = 0.0;
p1 = 0.0;
p2 = 0.0;

% feature extractor CASE1, independent assortment of fearure type and
% descriptor type
params.featureType = 'SURF';
params.descType = 'SURF';

% match method
params.matchMethod = 'DIRECTMATCH';

% Init success threshold
params.initSuccThr = 20;

% Organize some of the parameters
params.cameraParam = [ fx      0      cx; 
                       0       fy     cy; 
                       0       0      1 ];


params.distCoef = [k1, k2, p1, p2];


% --------------------------------------------------------------------------------------------
%  Matrix Parameters
% --------------------------------------------------------------------------------------------
params.kpNum = 2000;
params.mpIdLength = 11;

% --------------------------------------------------------------------------------------------
%  ORB Parameters
% --------------------------------------------------------------------------------------------

% ORB Extractor: Number of features per image
params.ORB_nDsecDim =  0; % define here if known or update in the initialization

% ORB Extractor: Number of features per image
params.ORB_nFeatures =  2000;

% ORB Extractor: Scale factor between levels in the scale pyramid 	
params.ORB_scaleFactor =  1.2;

% ORB Extractor: Number of levels in the scale pyramid	
params.ORB_nLevels =  8;

% ORB Extractor: Fast threshold
% Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
% Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
% You can lower these values if your images have low contrast		
params.ORB_iniThFAST =  20;
params.ORB_minThFAST =  7;
params.ORB_initFlag  = 0;

scaleFactorSet = zeros(1, params.ORB_nLevels);
scaleFactorSet(1, 1) = 1.0;
scaleFactorSigma2 = zeros(1, params.ORB_nLevels);
scaleFactorSigma2(1, 1) = 1.0;
for l = 2 : params.ORB_nLevels
    scaleFactorSet(1, l) = scaleFactorSet(1, l - 1) * params.ORB_scaleFactor;
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

