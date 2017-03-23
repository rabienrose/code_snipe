clc; clear all; close all

global coreDataMatrix;
addpath(genpath('../'))

config_lizao_m;

handler = createSLAMHandle();
init(handler);
startFrame = 1;
frameCount = 20;
bInit= false;
for frameId = startFrame:startFrame + frameCount - 1
    frameId 
    setImage(handler, frameId);
    extractFeatures(handler, frameId);
    if ~bInit
        bInit = tryInitPose(handler, frameId);
    else
%         coreDataMatrix =readGlobalMapFromC(handler);
%         idTransformation();
%         showTrack(2);
        predictPose(handler, frameId);
        searchByProjection(handler, frameId, 1);
        optimizePose(handler, frameId);
        %searchByProjection(handler, frameId, 20);         
        trackLocalMap(handler, frameId);
        optimizePose(handler, frameId);
        decisionKeyFrame(handler, frameId);
    end
    
%%  mapping without BA  
    if (frameId >= 3)
        coreDataMatrix =readGlobalMapFromC(handler);
  
        if(coreDataMatrix.frTypeMatrix(coreDataMatrix.frImgIdVec(frameId), 1) ~= 1)
           continue;
        end
         offSetFrame = 3;  
    %     if frameId < 8
    %     figure(1)
    %     set(gcf, 'position', [20, 600, 700, 300]);
    %     showTrack(frameId);   
    %     end
         searchByEpipolar(handler, frameId, offSetFrame);
    %     coreDataMatrix =readGlobalMapFromC(handler);
    %      if frameId < 8
    %     figure(2)
    %     set(gcf, 'position', [800, 600, 700, 300]);
    %     showTrack(frameId);
    %     drawnow;
    %     end


    %     figure(3)
    %     set(gcf, 'position', [20, 100, 700, 300]);
    %     showReprojection(frameId);    
    %     drawnow;

        triangulate(handler, frameId, offSetFrame); 

        coreDataMatrix = readGlobalMapFromC(handler);   
%         figure(4)
%     %     set(gcf, 'position', [800, 100, 700, 300]);  
%         showReprojection(frameId);
%         drawnow;
    end
end
destorySLAMHandle();
% coreDataMatrix.frCount = frameCount;
% showMapping(true);
