% this File generate all synthetic coreData
%   clc; close all; clear;
%% ---------------------------------------------------------------------------------------
%                                generate Synthetic data  
%-----------------------------------------------------------------------------------------

  addpath('../Configuration')
  
  global Dataparams;
  Dataconfig_default;
  
% generate data 
  [points3D, points2D, flag2D, viewpointsnum, AllR, Allt, K] = ...
                   Get3D2D2DPair(Dataparams);

% show the data
  visible = Dataparams.showtrajectory;
  frameID = Dataparams.ShowFrameId ;
  ShowData(points3D, points2D, flag2D, AllR, Allt, Dataparams.datarange, visible, frameID);
  
% image Width and height  
  frameId = Dataparams.currframeId;
  FramesNum = size(AllR, 2);
  PointsNum = size(points3D, 1);
  Image = uint8(255*ones(Dataparams.cameraparams.H, Dataparams.cameraparams.W));
  
% generate fake Descriptor (PointsNum <= 7000)
  fakeDescriptor = ChooseFakeDescriptor(PointsNum);
  
%-----------------------------------------------------------------------------------------
%                                can be used to fill the coreData  
%-----------------------------------------------------------------------------------------