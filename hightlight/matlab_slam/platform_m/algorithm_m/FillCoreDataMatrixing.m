% this File fill the all coreData
%   clc; close all; clear;

% for debug
% function [VisiblePositions, VisiblePointsNum, coreData] = FillCoreDataMatrixing(points3D, points2D, flag2D,...
%                                                  AllR, Allt, PointsNum, FramesNum, Image, fakeDescriptor)

  global coreData;   
     
%% ---------------------------------------------------------------------------------------
%                               Find the Track of 3D points  
%----------------------------------------------------------------------------------------- 
% reconstruct the flag data
  FlagMatrix = zeros(PointsNum, FramesNum);
  for FrameId = 1 : FramesNum
      FlagMatrix(:, FrameId) = flag2D{1, FrameId};
  end
  
% find Tracks of 3D points  
  for pId = 1 : PointsNum 
      currTrack = FindTrack(FlagMatrix(pId, :));
      TrackSet{pId, 1} = currTrack;
  end
  
% construct a matrix to indicata if a 2D point in a track
  PosiInTrackFlag = zeros(PointsNum, FramesNum);
  for posInTrackId = 1 : PointsNum
      PosiInTrackFlag(posInTrackId, TrackSet{posInTrackId,1}) = 1;  
  end
  
%% ---------------------------------------------------------------------------------------
%                               core data structure 
%-----------------------------------------------------------------------------------------  
  coreData = DataRoot1;
%   coreData.CurrentFrameId = frameId;
%   coreData.imgSize = [Dataparams.cameraparams.H, Dataparams.cameraparams.W];
  
% KFList structure  
  coreData.KFList = repmat(KeyFrameSlam1, FramesNum, 1);
  
% MPList structure
%   coreData.MPList = repmat(MapPointSlam, PointsNum, 1);

%% ---------------------------------------------------------------------------------------
%                               Fill the data of KeyFameSlam  
%----------------------------------------------------------------------------------------- 
VisiblePointsNum = [];
VisiblePositions = [];
  for frameNum = 1 : FramesNum
      CurrVisiblePointsNum = sum(FlagMatrix(:,frameNum));
      VisiblePointsNum = [VisiblePointsNum; CurrVisiblePointsNum];
      coreData.KFList(frameNum).Pose = [AllR{frameNum}, Allt(frameNum, :)'];
%       coreData.KFList(frameNum).KPList = repmat(KeyPointSlam, CurrVisiblePointsNum, 1);
      coreData.KFList(frameNum).Image = Image;
                  
      % get the positions of visible points   
      positions = find(FlagMatrix(:, frameNum)== 1);
      VisiblePositions{frameNum,1} = positions; 
  end

%% ---------------------------------------------------------------------------------------
%                               Fill the data of MapPointSlam   
%----------------------------------------------------------------------------------------- 
coreData.mpTrackMatrix = zeros(2, FramesNum, PointsNum);
coreData.mpIsBadVec = repmat('false',PointsNum, 1);
coreData.mpDescriptorMatrix = fakeDescriptor';
coreData.mpPosiMatrix = points3D';

  for mpNum = 1 : PointsNum  
      currTrackNum = size(TrackSet{mpNum, 1}, 2);
      index = 1;
      for frameNum = 1 : FramesNum
          if PosiInTrackFlag(mpNum, frameNum) == 1
              [Row, Col, value] = find(VisiblePositions{frameNum, :} == mpNum);
%               coreData.MPList(mpNum).Track(index).FrameId = frameNum;
%               coreData.MPList(mpNum).Track(index).KPId = Row;
              coreData.mpTrackMatrix(1, index, mpNum) = frameNum;
              coreData.mpTrackMatrix(2, index, mpNum) = Row;
              index = index + 1;
          end
      end
  end
  

%% ---------------------------------------------------------------------------------------
%                               Fill the data in KeyPointSlam  
%-----------------------------------------------------------------------------------------  

  for frameNum = 1 : FramesNum
      for mpNum = 1 : VisiblePointsNum(frameNum)
           CurrVisibleID = VisiblePositions{frameNum, 1}(mpNum);
%            coreData.KFList(frameNum).KPList(mpNum).mpId = CurrVisibleID;
           coreData.KFList(frameNum).kpMpIdVec(mpNum, 1) = CurrVisibleID;
%            if PosiInTrackFlag(CurrVisibleID, frameNum) == 1 
%                 for TrackNum = 1 : size(coreData.MPList(CurrVisibleID).Track, 1) 
%                 for TrackNum = 1 : size(coreData.mpTrackMatrix, 1)
%                      [InterSec, ia, ib] = intersect(frameNum, coreData.MPList(CurrVisibleID).Track(TrackNum, 1).FrameId);
%                      if InterSec == frameNum
%                             coreDataTruth.KFList(frameNum).KPList(mpNum).posiInTrack = TrackNum;
%                      end
%                 end
%            end
%            coreData.KFList(frameNum).KPList(mpNum).posi = points2D{frameNum}(CurrVisibleID,1:2);           
           coreData.KFList(frameNum).kpPosiMatrix(:, mpNum) = points2D{frameNum}(CurrVisibleID, 1:2)';
%            coreData.KFList(frameNum).KPList(mpNum).descriptor = fakeDescriptor(mpNum, :);
           coreData.KFList(frameNum).kpDescriptorMatrix(:, mpNum) = fakeDescriptor(mpNum, :)';
%            coreData.KFList(frameNum).KPList(mpNum).octave = 1;
           coreData.KFList(frameNum).kpOctaveVec(mpNum, 1) = 1;
      end
  end
  
% end
  
% keep core data
% use W H Theta to generate camera Intrinsic parameters and 
% use FlagMatrix to choose Raw Features to display

%   clearvars -except coreData W H Theta FlagMatrix;


