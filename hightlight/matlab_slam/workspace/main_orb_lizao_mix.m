clc; clear all; close all

global coreDataMatrix;
addpath(genpath('../'))
config_lizao_m;

handler = createSLAMHandle();
init(handler, '/Users/test/roadDBCore/algorithm/v2.0/config/config.xml');
pGMap = getGMapPtr(handler);
startFrame = 1;
frameCount = 10;
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
       
   % ------------ test part-------------------------------
%     if frameId > 3
        predictPose(handler, frameId);
%         %  matlab function
%     else
%         coreDataMatrix = readGlobalMapFromC(handler);
        %dTransforToM(); 
        %predictPose_m(frameId);
        %idTransforToC();
%       readGlobalMapFromCWithName(handler, '/Users/test/Desktop/lib/core.mat');
%         writeGlobalMapToC(handler, coreDataMatrix);
% %     end
        
   % ----------------------------------------------------- 
        coreDataMatrix = readGlobalMapFromC(handler);
%         idTransforToM();
%         idTransforToC();
%         writeGlobalMapToC(handler, coreDataMatrix);
        
        searchByProjection(handler, frameId, 1);
        coreDataMatrix = readGlobalMapFromC(handler);
        optimizePose(handler, frameId); 
        coreDataMatrix = readGlobalMapFromC(handler);
        trackLocalMap(handler, frameId);
        coreDataMatrix = readGlobalMapFromC(handler);
        optimizePose(handler, frameId);
        coreDataMatrix = readGlobalMapFromC(handler);
        decisionKeyFrame(handler, frameId);
        coreDataMatrix = readGlobalMapFromC(handler);

    end
    
%%  mapping without BA  
    if (frameId >= 3)
        pFrame = getFrame(pGMap, frameId);
        fType = getFType(pFrame)
        if fType ~= 1
            continue;
        end
        offSetFrame = 3;  
        searchByEpipolar(handler, frameId, offSetFrame);
        triangulate(handler, frameId, offSetFrame); 
        
%         coreDataMatrix =readGlobalMapFromC(handler);   
%         SetMPsToVisPlat(handler, 0);
%         SetKPsToVisPlat(handler, 0);
%         SaveVisFile(handler);
%         showReprojection(frameId);    
%         drawnow;
    end
end
% destorySLAMHandle();
% coreDataMatrix.frCount = frameCount;
% showMapping(true);
    %     if frameId < 8
    %     figure(1)
    %     set(gcf, 'position', [20, 600, 700, 300]);
    %     showTrack(frameId);   
    %     end
    %      if frameId < 8
    %     figure(2)
    %     set(gcf, 'position', [800, 600, 700, 300]);
    %     showTrack(frameId);
    %     drawnow;
    %     end
%         figure(4)
    %     set(gcf, 'position', [800, 100, 700, 300]);  
%         showReprojection(frameId);
% %         drawnow; 
    %     figure(3)
    %     set(gcf, 'position', [20, 100, 700, 300]);
    %     showReprojection(frameId);    
    %     drawnow;