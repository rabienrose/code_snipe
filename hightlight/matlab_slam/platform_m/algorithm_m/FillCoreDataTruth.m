% This File fill the all coreData
%   clc; close all; clear;


  global coreDataTruth;   
     
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
  coreDataTruth = DataRoot;
  coreDataTruth.CurrentFrameId = frameId;
  coreDataTruth.imgSize = [Dataparams.cameraparams.H, Dataparams.cameraparams.W];
  
% KFList structure  
  coreDataTruth.KFList = repmat(KeyFrameSlam, FramesNum, 1);
  
% MPList structure
  coreDataTruth.MPList = repmat(MapPointSlam, PointsNum, 1);

%% ---------------------------------------------------------------------------------------
%                               Fill the data of KeyFameSlam  
%----------------------------------------------------------------------------------------- 
VisiblePointsNum = [];
VisiblePositions = [];
  for frameNum = 1 : FramesNum
      CurrVisiblePointsNum = sum(FlagMatrix(:,frameNum));
      VisiblePointsNum = [VisiblePointsNum; CurrVisiblePointsNum];
      coreDataTruth.KFList(frameNum).Pose = [AllR{frameNum}, Allt(frameNum, :)'];
      coreDataTruth.KFList(frameNum).KPList = repmat(KeyPointSlam, CurrVisiblePointsNum, 1);
      coreDataTruth.KFList(frameNum).Image = Image;
                  
      % get the positions of visible points   
      positions = find(FlagMatrix(:, frameNum)== 1);
      VisiblePositions{frameNum,1} = positions; 
  end

%% ---------------------------------------------------------------------------------------
%                               Fill the data of MapPointSlam   
%-----------------------------------------------------------------------------------------  
  for mpNum = 1 : PointsNum
      coreDataTruth.MPList(mpNum).Posi = points3D(mpNum, :);
      currTrackNum = size(TrackSet{mpNum, 1}, 2);
      coreDataTruth.MPList(mpNum).Track = repmat(TrackItem, currTrackNum, 1);
      index = 1;
      for frameNum = 1 : FramesNum
          if PosiInTrackFlag(mpNum, frameNum) == 1
              [Row, Col, value] = find(VisiblePositions{frameNum, :} == mpNum);
              coreDataTruth.MPList(mpNum).Track(index).FrameId = frameNum;
              % relative position
              coreDataTruth.MPList(mpNum).Track(index).KPId = Row;
              index = index + 1;
          end
      end
      coreDataTruth.MPList(mpNum).Descriptor = fakeDescriptor(mpNum, :);
%        coreDataTruth.MPList{frameNum}.isBad = ;
  end

%% ---------------------------------------------------------------------------------------
%                               Fill the data in KeyPointSlam  
%-----------------------------------------------------------------------------------------  

  for frameNum = 1 : FramesNum
      for mpNum = 1 : VisiblePointsNum(frameNum)
           CurrVisibleID = VisiblePositions{frameNum, 1}(mpNum);
           coreDataTruth.KFList(frameNum).KPList(mpNum).mpId = CurrVisibleID;
           if PosiInTrackFlag(CurrVisibleID, frameNum) == 1 
                for TrackNum = 1 : size(coreDataTruth.MPList(CurrVisibleID).Track, 1)                     
                     [InterSec, ia, ib] = intersect(frameNum, coreDataTruth.MPList(CurrVisibleID).Track(TrackNum, 1).FrameId);
%                      % save the TrackItem number 
%                      if InterSec == frameNum
%                             coreDataTruth.KFList(frameNum).KPList(mpNum).posiInTrack = TrackNum;
%                      end
                end
           end
           coreDataTruth.KFList(frameNum).KPList(mpNum).posi = points2D{frameNum}(CurrVisibleID,1:2);
           coreDataTruth.KFList(frameNum).KPList(mpNum).descriptor = fakeDescriptor(mpNum, :);
           coreDataTruth.KFList(frameNum).KPList(mpNum).octave = 1;
      end
  end
  

  
% keep core data
% use W H Theta to generate camera Intrinsic parameters and 
% use FlagMatrix to choose Raw Features to display

%   clearvars -except coreDataTruth W H Theta FlagMatrix;


