% Fill the coreDataMatrix with some options
%   clc; close all; clear; 
%   addpath('../CoreData')
  global coreDataMatrix;  
  global Dataparams;
     
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
  
% get the positions of visible points   
  VisiblePositions = [];
  for frameNum = 1 : FramesNum                    
      positions = find(FlagMatrix(:, frameNum)== 1);
      VisiblePositions{frameNum,1} = positions; 
  end
%% ---------------------------------------------------------------------------------------
%                               core data structure 
%-----------------------------------------------------------------------------------------  
  coreDataMatrix = coreDataStructure;

% intial the Data
  mpNum = Dataparams.datanumber;
  kpNum = mpNum;
  mpIdLength = 11;
  descDim = 32;
  frameCount = size(Allt,1);
  coreDataMatrix.mpDescMatrix = cast(zeros(descDim, mpNum), 'uint8');
  coreDataMatrix.kpDescMatrix = cast(zeros(descDim, kpNum, frameCount), 'uint8');
   
  coreDataMatrix.mpPosiMatrix = zeros(3, mpNum);
  coreDataMatrix.mpTrackMatrix= zeros(2, frameCount, mpNum);
  coreDataMatrix.mpIsBadVec = 0 * ones(mpNum, 1);
  coreDataMatrix.mpTracksCountVec = zeros(mpNum, 1);
    
  coreDataMatrix.frPoseMatrix = zeros(3, 4, frameCount);
  coreDataMatrix.frTypeMatrix = ones(frameCount, 1);
  coreDataMatrix.frImgIdVec = zeros(frameCount, 1);
  coreDataMatrix.frImgDirMatrix = Dataparams.imagePaths.ImageLocation(1, 1:FramesNum)';
  coreDataMatrix.frCount = frameCount;
    
  coreDataMatrix.kpPosiMatrix = zeros(2, kpNum, frameCount);
  coreDataMatrix.kpMpIdMatrix =[zeros(kpNum, 1, frameCount), ...
                                  0 * ones(kpNum, mpIdLength - 1, frameCount)];
  coreDataMatrix.kpOctaveMatrix = zeros(kpNum, frameCount);
  coreDataMatrix.kpsCountVec = zeros(frameCount, 1);
    
%   coreDataMatrix.imgSize = [Dataparams.cameraparams.W, Dataparams.cameraparams.H];

%% ---------------------------------------------------------------------------------------
%                               Fill the data of mp  
%----------------------------------------------------------------------------------------- 

% mpPosiMatrix (3 * MP)
  coreDataMatrix.mpPosiMatrix = points3D';
  
% mpisBadVec (MP * 1) 
% set as default
  
% mpDescMatrix (N * MP)
  coreDataMatrix.mpDescMatrix = fakeDescriptor';

% mpTrackMatrix (2 * trackLength * MP) & mpTracksCountVec
    for mpIndex = 1 : PointsNum  
      currTrackNum = size(TrackSet{mpIndex, 1}, 2);
      coreDataMatrix.mpTracksCountVec(mpIndex, 1) = currTrackNum;
      index = 1;
      for frameId = 1 : FramesNum
          if PosiInTrackFlag(mpIndex, frameId) == 1
%               [kpId, Col, value] = find(VisiblePositions{frameId, :} == mpIndex);
              coreDataMatrix.mpTrackMatrix(1, index, mpIndex) = frameId;
              coreDataMatrix.mpTrackMatrix(2, index, mpIndex) = mpIndex;    % mpIndex and kpIndex are in the same order
              index = index + 1;
          end
      end
    end
  
% mpCount (1 * 1)
  coreDataMatrix.mpCount = mpNum;

%% ---------------------------------------------------------------------------------------
%                               Fill the data of fr  
%----------------------------------------------------------------------------------------- 

% frPosMatrix (3 * 4 * frame)
  for frameInd = 1 : frameCount
        coreDataMatrix.frPoseMatrix(:, :, frameInd) = [AllR{1, frameInd}, Allt(frameInd, :)'];
  end
% frImgDirMatrix (frame * length(dir))
% here set as default

% frTypeMatrix (frame * length(Type) -> '0' for normal frame, '1'for Key frame, '2' for bad key frame)
% default-> 'key frame'
  for frameInd = 1 : frameCount
        coreDataMatrix.frTypeMatrix(frameInd, 1) = 1;
  end
% frImgIdVec (frame * 1) % to find the position of a picture in the compute files %
  coreDataMatrix.frImgIdVec = [1 : frameCount]';
 

%% ---------------------------------------------------------------------------------------
%                               Fill the data of kp   
%-----------------------------------------------------------------------------------------  
% kpsCountVec (frame * 1)
  coreDataMatrix.kpsCountVec = kpNum * ones(FramesNum, 1);
  
% kpPosiMatrix (2 * KP * frame)
  for frInd = 1 : frameCount
        coreDataMatrix.kpPosiMatrix(:, 1 : coreDataMatrix.kpsCountVec(frInd, 1), frInd) = ...
            points2D{1, frInd}(:, 1:2)';
  end  

% kpMpIdMatrix (KP * (1 + 10) * frame)
  for frInd = 1 : frameCount
       countInd = 1;
       for kpInd = 1 : kpNum
            CurrVisibleID = VisiblePositions{frInd, 1}(countInd);
            % here no kp vs multiple mps exist, so we set it to 1
            if (CurrVisibleID == kpInd)
                coreDataMatrix.kpMpIdMatrix(kpInd, 1, frInd) = 1;
                coreDataMatrix.kpMpIdMatrix(kpInd, 2, frInd) = CurrVisibleID;
                countInd = countInd + 1;
            end   
            if countInd > size(VisiblePositions{frInd, 1},1)
               countInd = 1;
               continue
            end
       end
       
  end

% kpOctaveMatrix (KP * frame) 
%   coreDataMatrix.kpOctaveMatrix = ones(kpNum, frameCount);
  
  
% kpDescMatrix (N * KP * frame)
  for frInd = 1 : frameCount
        coreDataMatrix.kpDescMatrix(:, 1 : coreDataMatrix.kpsCountVec(frInd, 1), frInd) = ...
            fakeDescriptor(:, :)';
  end
 
%-----------------------------------------------------------------------------------------  

  


