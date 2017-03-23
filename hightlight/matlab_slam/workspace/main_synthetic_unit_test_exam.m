% An example of how to use synthetic data to do unit test
%%
  clc; clear all; close all

  addpath(genpath('../'))
  global coreDataMatrix;
  global coreDataMatrixCompare;
  global handler;
  global pGMap;
   
 % choose testcase
  testcase = 2;
  
%% Case 1: noiseless case
%  step one: generate full Synthetic data
%  after generating data, you need to manually pick up the data you want 
%  and save to the path. i.e. '/Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat'
if testcase == 1
  rmpath('../platform/bin')   % remove the path of the same name function "extractFeatures.mexmaci64"
  TestCoreData;
  coreDataMatrixCompare = coreDataMatrix;
  upToFrame = 3;
  pickDataUpFrame;
  
  % clear information: frameType, kpMpId
  coreDataMatrix.frTypeMatrix(upToFrame, 1) = 0;
  coreDataMatrix.kpMpIdMatrix(:, :, upToFrame) = zeros(2000, 11, 1);
  
  clearvars -except  coreDataMatrix  coreDataMatrixCompare VisiblePositions K testcase;
  row123 = intersect(intersect(VisiblePositions{1,1}, VisiblePositions{2,1}), VisiblePositions{3,1});
%   save /Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat coreDataMatrix
%   save /Users/test/roadDBCore/dataset/SyntheticTest/SyntheticTotal.mat coreDataMatrixCompare row123
 
% Step two: union test searchByProjection (Noiseless case)
  addpath('../platform/bin')

  destorySLAMHandle();
  handler = createSLAMHandle();
% <ImageWidth>1241</ImageWidth> and <ImageHeight>376</ImageHeight> should
% be added to the config.xml
  init(handler, '/Users/test/roadDBCore/algorithm/v2.0/config/config.xml');
  pGMap = getGMapPtr(handler);
   
% ------------ test1: test searchByProjection func begin ------------------      
% ---- method one ---- the mxDestroyArray part should be commented in
%                      MapIO.cpp for this method
%   writeGlobalMapToCWithPath(handler, '/Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat');
% ---- method two ----
  writeGlobalMapToC(handler, coreDataMatrix);
  frameId = 3;
  changeCameraParam(handler, 620.5, 620.5, 620.5, 188);
  searchByProjection(handler, frameId, 1);
  coreDataMatrix = readGlobalMapFromC(handler);
  
  % compare the result(row123 & row_after_searchBy) 
  row_after_searchBy = find(coreDataMatrix.kpMpIdMatrix(:, 1, coreDataMatrix.frImgIdVec(frameId)) > 0);

end   
% ------------ test1: end -------------------------------------------------     


%% Case 2: noisy case
%  step one: generate full Synthetic data
%  after generating data, you need to manually pick up the data you want 
%  and save to the path. i.e. '/Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat'
if testcase == 2
  rmpath('../platform/bin')   % remove the path of the same name function "extractFeatures.mexmaci64"
  TestCoreData;
  coreDataMatrixCompare = coreDataMatrix;
  upToFrame = 3;
  pickDataUpFrame;
  
  % clear information: frameType, kpMpId
  coreDataMatrix.frTypeMatrix(upToFrame, 1) = 0;
  coreDataMatrix.kpMpIdMatrix(:, :, upToFrame) = zeros(2000, 11, 1);
  % add a small drift angle (degree) in the direction of {x-axis, y-axis ,z-axis} as noise to the Pose 
   driftangle = [0, 0, 0];
   % drift angle in the x-axis
   theta_x = driftangle(1) * pi / 180;
   Rx = [             1,             0,             0
                      0,  cos(theta_x),  -sin(theta_x)
                      0,  sin(theta_x),   cos(theta_x)];                      
   % drift angle in the y-axis
   theta_y = driftangle(2) * pi / 180;
   Ry = [  cos(theta_y),             0,   sin(theta_y)
                      0,             1,             0
         -sin(theta_y),             0,   cos(theta_y)];         
  % drift angle in the z-axis
  theta_z = driftangle(3) * pi / 180;
  Rz = [  cos(theta_z), -sin(theta_z),             0
          sin(theta_z),  cos(theta_z),             0
                     0,             0,             1];  
  frameId = 3;                     
  coreDataMatrix.frPoseMatrix(:, 1:3, coreDataMatrix.frImgIdVec(frameId)) = ...
      coreDataMatrix.frPoseMatrix(:, 1:3, coreDataMatrix.frImgIdVec(frameId)) * Rx * Ry * Rz;
  
  clearvars -except  coreDataMatrix  coreDataMatrixCompare VisiblePositions K step;
  row123 = intersect(intersect(VisiblePositions{1,1}, VisiblePositions{2,1}), VisiblePositions{3,1});
%   save /Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat coreDataMatrix
%   save /Users/test/roadDBCore/dataset/SyntheticTest/SyntheticTotal.mat coreDataMatrixCompare row123
  
% Step two: union test searchByProjection (Noisy case)
  addpath('../platform/bin')

  destorySLAMHandle();
  handler = createSLAMHandle();
  init(handler, '/Users/test/roadDBCore/algorithm/v2.0/config/config.xml');
  pGMap = getGMapPtr(handler);
   
% ------------ test2: test searchByProjection func begin ------------------

% ---- method one ---- the mxDestroyArray part should be commented in
%                      MapIO.cpp for this method
%   writeGlobalMapToCWithPath(handler, '/Users/test/roadDBCore/dataset/SyntheticTest/Synthetic.mat');
% ---- method two ----
  writeGlobalMapToC(handler, coreDataMatrix);
  frameId = 3;
  changeCameraParam(handler, 620.5, 620.5, 620.5, 188);
  searchByProjection(handler, frameId, 1);
  coreDataMatrix = readGlobalMapFromC(handler);
  
  % compare the result(row123 & row_after_searchBy) 
  row_after_searchBy = find(coreDataMatrix.kpMpIdMatrix(:, 1, coreDataMatrix.frImgIdVec(frameId)) > 0);
  
% ------------ test2: end -------------------------------------------------  
end

