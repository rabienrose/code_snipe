clear all;
addpath(genpath('bin'));
confRootDir = '/Volumes/chamo/working/matlab_slam/v2.0/config/';
confDir = [confRootDir 'test_cases.xml'];
imagRoot = '/Volumes/chamo/dataset/Vehicle_SLAM_Video_Test_Set/';
curCaseName = 'Brimley_Loop_No_GPS_1';
handler = CreateTestCaseParser(confDir,imagRoot);
imgDir = GetImagDir(handler, curCaseName, 2)
[startF endF] = GetFrameRange(handler, curCaseName)
config = GetConfigDir(handler, curCaseName)
[fx fy cx cy] = GetCamConfig(handler, curCaseName)
caseNames = GetCaseNames(handler)