% clear all; clc; close all;
%  load coreData1000P360F.mat
%% Test the consistency of coreData
addpath(genpath('../../'))
% remove the path of the same name function "extractFeatures.mexmaci64"
  rmpath('../../platform/bin') 
warning off all
% profile off
% profile clear
% profile on
%% ---------------------------------------------------------------------------------------
%                                generate Synthetic data  
%-----------------------------------------------------------------------------------------  
% generate all data {3D, 2D points, R, t, K, descriptor}  
  GenerateAllData;
  
% fill the pure coreDataTruth / with options(i.e, add noise, fill part of the data) 
%   FillCoreDataTruth;
%   FillCoreDataWithOptions;

% fill the coreData in Matrix formation
  FillCoreDataWithOptionsMatrixing;

% set cameraparams (for function 'ShowReprojection')

  global params;
  params.cameraParam = K;
  
%   clearvars -except  coreDataMatrix  VisiblePositions K;% coreDataTruth params;


 %%
  FrameId = 2;

% show data
% coreDataType: choose the pure coreData ('1') or noisy coreData ('2')
% % show the track for two frames (FrameID - 1, FrameID)
  figure(88)
  showTrack_t(FrameId)  

% % display visible 2D points
%   figure(89)
%   showRawFeature_t(FrameId)  
 
% show the trajectory
%   figure(90)  
%   showMapping_t(true)
 
% show the reprojection and corresponding error
  figure(91)
  showReprojection_t(FrameId-1)
 
% show specific camera pose and corresponding 3D cloud
%   figure(92)
%   showMapEach_t(FrameId)

% clk = clock;
% time = strcat(num2str(clk(1)), num2str(clk(2)), num2str(clk(3)),...
%               num2str(clk(4)), num2str(clk(5)), num2str(round(clk(6))));
% profsave(profile('info'), strcat('/users/test/desktop/speeduptest/profile_results_showTrackcoreDataMatrixing2000P720F', time));

