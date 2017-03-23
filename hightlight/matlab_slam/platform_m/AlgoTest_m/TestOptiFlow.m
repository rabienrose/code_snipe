close all
clear all
clc

%% Add path
addpath('../AlgoLib/OptiFlow');
addpath('../MianFlow');

%% global parameters from config
global params;
config;

%% Load Configuration
cameraParam = params.cameraParam;
distCoef    = params.distCoef;
featureType = params.featureType;
descType    = params.descType;

%% Init Tracker
configPath = '/Users/zhaiq/Documents/ygomi/project/roadDBCore/SLAM_Matlab_Version/Tracker/config/KITTI00-02.yaml';

%% Get a List of All Image File Names in a Directory.
%imageDir = '/Users/test/Data/test';
imageDir = '/Volumes/chamo/dataset/KITTI/00/image_0';
imagePaths = imageSet(imageDir);

%% Compute Initial Camera Pose and 3D Points
I1 = read(imagePaths, 2);
I2 = read(imagePaths, 3);
if size(I1, 3) > 1
    I1 = rgb2gray(I1);
    I2 = rgb2gray(I2);
end

% Get the camera Params
IntrinsicMatrix = cameraParam';
cameraParams = cameraParameters('IntrinsicMatrix', IntrinsicMatrix);

% Detect key points with assigned method
points1 = DetectKeyPoints(I1, featureType);
points2 = DetectKeyPoints(I2, featureType);

% Extract features of the detected key points with assigned method
descriptor1 = ExtractFeatures(I1, points1, descType);
descriptor2 = ExtractFeatures(I2, points2, descType);
points1_c.Location = points1.Location;
points1_c.Scale = points1.Scale;
points1_c.Metric = points1.Metric;
points2_c.Location = points2.Location;
points2_c.Scale = points2.Scale;
points2_c.Metric = points2.Metric;

%%
[ssss ss] = OptiFlow(I1, I2, points1_c, points2_c, descriptor1, descriptor2,params.cameraParam);





