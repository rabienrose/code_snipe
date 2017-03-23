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
testConfig;
caseCount = size(TestSet, 2);
for i=1:3
    clear coreDataMatrix;
    configname = ['../../config/' TestSet(i).configName];
    imageDir = TestSet(i).imag;
    config_zili;
    destorySLAMHandle();
    handler = createSLAMHandle();
    init(handler, configname);
    pGMap = getGMapPtr(handler);
    bInit= false;
    for frameId = TestSet(i).startFrame:TestSet(i).endFrame
        
        imageAddr = getImagFullName(TestSet(i).fileNameLen, TestSet(i).imag, frameId);
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
            frameId
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
    coreDataMatrix = readGlobalMapFromC(handler);
    save(['../../../output/' TestSet(i).name '.mat'],'coreDataMatrix');
end
