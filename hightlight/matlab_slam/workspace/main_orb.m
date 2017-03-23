clear all
close all
clc
global coreDataMatrix;
global pGMap;
global handler;
addpath(genpath('../platform'));
addpath(genpath('../visualization'));
addpath(genpath('../tools'));
addpath(genpath('../platform_m'));
config_zili;
destorySLAMHandle();
handler = createSLAMHandle();
init(handler, '../../config/config_iphone.xml');
pGMap = getGMapPtr(handler);
startFrame = 1;
bInit= false;
for frameId = startFrame:startFrame+500
    frameId
    intStr = '000000';
    intStrN = num2str(frameId);
    intStr(end-size(intStrN,2)+1:end) = intStrN;
    imageAddr = ['/Volumes/chamo/dataset/KITTI/00/image_0/' intStr '.png'];
    setImage(handler, frameId,imageAddr);
    extractFeatures(handler, frameId);
    if ~bInit
        bInit = tryInitPose(handler, frameId);
    else
        %coreDataMatrix = readGlobalMapFromC(handler);
        %predictPose_m(frameId);
        %writeGlobalMapToC(handler, coreDataMatrix);
        
        predictPose(handler, frameId);
        searchByProjection(handler, frameId, 1);
        optimizePose(handler, frameId);
        trackLocalMap(handler, frameId);
        optimizePose(handler, frameId);
        decisionKeyFrame(handler, frameId);
        pFrame = getFrame(pGMap, frameId);
        fType = getFType(pFrame);
        if fType ~= 1
            continue;
        end
        offSetFrame = 3;  
        searchByEpipolar(handler, frameId, offSetFrame);
        triangulate(handler, frameId, offSetFrame);
        localBA(handler, frameId, 3);
        
        ShowMPs_C(frameId);
        SetMPsToVisPlat(handler, 0, -1);
        SetKPsToVisPlat(handler, 0);
        SaveVisFile(handler, '../../../output/allData.chamo');
        drawnow;
    end
end
