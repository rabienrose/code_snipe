% Fill the coreData with some options
%   clc; close all; clear; 
  addpath('../CoreData')
  global coreData;   
     
%% ---------------------------------------------------------------------------------------
%                                Add noise to the pure data  
%-----------------------------------------------------------------------------------------  
% add noise to the 3D points
  if Dataparams.addnoise3dposi == 1
        noisetype = Dataparams.noisetype;
        switch noisetype
            case 'Normal'
                mu = Dataparams.Normalnoise.mu;
                std = Dataparams.Normalnoise.std;
                noise3ddata = random('Normal', mu, std, PointsNum, 3);
            case 'Uniform'
                a = Dataparams.Uniformnoise.a;
                b = Dataparams.Uniformnoise.b;
                noise3ddata = random('Uniform', a, b, PointsNum, 3);
            case 'Gamma'
                a = Dataparams.Gammanoise.a;
                b = Dataparams.Gammanoise.b;              
                noise3ddata = random('Gamma', a, b, PointsNum, 3);    
            otherwise
               disp('Unexpected noise type.');       
        end
        points3D = points3D + noise3ddata;
  end

 
% add noise to the 2D points of specific frames
  if Dataparams.addnoise2dposi == 1
        noisy2dframe = Dataparams.addnoise2dframesId;   
        noisetype = Dataparams.noisetype;
        switch noisetype
            case 'Normal'
                mu = Dataparams.Normalnoise.mu;
                std = Dataparams.Normalnoise.std;
                for noisy2dframeId = 1 : size(noisy2dframe, 2)
                        noise2ddata = random('Normal', mu, std, PointsNum, 2);
                        points2D{noisy2dframe(1, noisy2dframeId)} = ...
                          points2D{noisy2dframe(1, noisy2dframeId)}(:, 1:2) + noise2ddata;  
                end
            case 'Uniform'
                a = Dataparams.Uniformnoise.a;
                b = Dataparams.Uniformnoise.b;
                for noisy2dframeId = 1 : size(noisy2dframe, 2)
                        noise2ddata = random('Uniform', a, b, PointsNum, 2);
                        points2D{noisy2dframe(1, noisy2dframeId)} = ...
                          points2D{noisy2dframe(1, noisy2dframeId)}(:, 1:2) + noise2ddata;  
                end
            case 'Gamma'
                a = Dataparams.Gammanoise.a;
                b = Dataparams.Gammanoise.b;              
                for noisy2dframeId = 1 : size(noisy2dframe, 2)
                        noise2ddata = random('Gamma', a, b, PointsNum, 2);
                        points2D{noisy2dframe(1, noisy2dframeId)} = ...
                          points2D{noisy2dframe(1, noisy2dframeId)}(:, 1:2) + noise2ddata;  
                end
            otherwise
               disp('Unexpected noise type.');       
        end
  end

% add noise to the descriptor
  if Dataparams.addnoisedescriptor == 1
        noisetype = Dataparams.noisetype;
        [row, col] = size(fakeDescriptor);
        switch noisetype
            case 'Normal'
                mu = Dataparams.Normalnoise.mu;
                std = Dataparams.Normalnoise.std;
                noisedesdata = random('Normal', mu, std, row, col);
            case 'Uniform'
                a = Dataparams.Uniformnoise.a;
                b = Dataparams.Uniformnoise.b;
                noisedesdata = random('Uniform', a, b, row, col);
            case 'Gamma'
                a = Dataparams.Gammanoise.a;
                b = Dataparams.Gammanoise.b;              
                noisedesdata = random('Gamma', a, b, row, col);    
            otherwise
               disp('Unexpected noise type.');       
        end
        fakeDescriptor = fakeDescriptor + noisedesdata;
  end  

% add a small rotation (drift angle) as noise to the Pose[R, t] of specific frames
 % add 'noise' to the R 
   if Dataparams.addnoise2poseR == 1
        driftangle = Dataparams.addposedriftangle;
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
        
        % add rotation(noise) to the original pose
        noisyposeframe = Dataparams.addnoisePoseRframesId; 
        for noisyposeframeId = 1 : size(noisyposeframe, 2)
               AllR{noisyposeframe(1, noisyposeframeId)} = ...
                   AllR{noisyposeframe(1, noisyposeframeId)} * Rx * Ry * Rz;  
        end 
   end
   
 % add noise to the t
   if Dataparams.addnoise2poset == 1
        noisetype = Dataparams.noisetype;
        noisetNum = size(Dataparams.addnoisePosetframesId, 2);
        switch noisetype
            case 'Normal'
                mu = Dataparams.Normalnoise.mu;
                std = Dataparams.Normalnoise.std;
                noisetdata = random('Normal', mu, std, noisetNum, 3);
            case 'Uniform'
                a = Dataparams.Uniformnoise.a;
                b = Dataparams.Uniformnoise.b;
                noisetdata = random('Uniform', a, b, noisetNum, 3);               
            case 'Gamma'
                a = Dataparams.Gammanoise.a;
                b = Dataparams.Gammanoise.b;              
                noisetdata = random('Gamma', a, b, noisetNum, 3);    
            otherwise
               disp('Unexpected noise type.');       
        end
        Allt(Dataparams.addnoisePosetframesId, :) = ...
            Allt(Dataparams.addnoisePosetframesId, :) + noisetdata;
   end  

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
  coreData = DataRoot;
  coreData.CurrentFrameId = frameId;
  coreData.imgSize = [Dataparams.cameraparams.H, Dataparams.cameraparams.W];
  
% KFList structure  
  coreData.KFList = repmat(KeyFrameSlam, FramesNum, 1);
  
% MPList structure
  coreData.MPList = repmat(MapPointSlam, PointsNum, 1);

%% ---------------------------------------------------------------------------------------
%                               Fill the data of KeyFameSlam  
%----------------------------------------------------------------------------------------- 
VisiblePointsNum = [];
VisiblePositions = [];
% generate indicate parameters
for frameNum = 1 : FramesNum
      CurrVisiblePointsNum = sum(FlagMatrix(:,frameNum));
      VisiblePointsNum = [VisiblePointsNum; CurrVisiblePointsNum];
    % get the positions of visible points   
      positions = find(FlagMatrix(:, frameNum)== 1);
      VisiblePositions{frameNum,1} = positions; 
end

% fill the data to the specific frame 'EndframeNum'
EndframeNum = Dataparams.filluptoframe;

  for frameNum = 1 : EndframeNum
      CurrVisiblePointsNum = sum(FlagMatrix(:,frameNum));
      coreData.KFList(frameNum).Pose = [AllR{frameNum}, Allt(frameNum, :)'];
      coreData.KFList(frameNum).KPList = repmat(KeyPointSlam, CurrVisiblePointsNum, 1);
      coreData.KFList(frameNum).Image = Image;
  end

%% ---------------------------------------------------------------------------------------
%                               Fill the data of MapPointSlam   
%-----------------------------------------------------------------------------------------  
  for mpNum = 1 : PointsNum
      % fill the 3D points or not (according to the parameter: Dataparams.fill3dposi)
      if Dataparams.fill3dposi == 1
           coreData.MPList(mpNum).Posi = points3D(mpNum, :);
      end
      currTrackNum = size(TrackSet{mpNum, 1}, 2);
      coreData.MPList(mpNum).Track = repmat(TrackItem, currTrackNum, 1);
      index = 1;
      for frameNum = 1 : EndframeNum
          if PosiInTrackFlag(mpNum, frameNum) == 1
              [Row, Col, value] = find(VisiblePositions{frameNum, :} == mpNum);
              coreData.MPList(mpNum).Track(index).FrameId = frameNum;
              % relative position
              coreData.MPList(mpNum).Track(index).KPId = Row;
              index = index + 1;
          end
      end
      coreData.MPList(mpNum).Descriptor = fakeDescriptor(mpNum, :);
%        coreData.MPList{frameNum}.isBad = ;
  end

%% ---------------------------------------------------------------------------------------
%                               Fill the data in KeyPointSlam  
%-----------------------------------------------------------------------------------------  
  for frameNum = 1 : EndframeNum
      for mpNum = 1 : VisiblePointsNum(frameNum)
           CurrVisibleID = VisiblePositions{frameNum, 1}(mpNum);
           coreData.KFList(frameNum).KPList(mpNum).mpId = CurrVisibleID;
           if PosiInTrackFlag(CurrVisibleID, frameNum) == 1 
                for TrackNum = 1 : size(coreData.MPList(CurrVisibleID).Track, 1)                     
                     [InterSec, ia, ib] = intersect(frameNum, coreData.MPList(CurrVisibleID).Track(TrackNum, 1).FrameId);
                     % save the TrackItem number 
                   %  if InterSec == frameNum
                   %        coreData.KFList(frameNum).KPList(mpNum).posiInTrack = TrackNum;
                   %  end
                end
           end
           coreData.KFList(frameNum).KPList(mpNum).posi = points2D{frameNum}(CurrVisibleID,1:2);
           coreData.KFList(frameNum).KPList(mpNum).descriptor = fakeDescriptor(mpNum, :);
           coreData.KFList(frameNum).KPList(mpNum).octave = 1;
      end
  end
  

  
% keep core data
% use W H Theta to generate camera Intrinsic parameters and 
% use FlagMatrix to choose Raw Features to display

%   clearvars -except coreData W H Theta FlagMatrix;


