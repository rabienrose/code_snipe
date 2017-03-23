% Author: Zili Wang
% Date: 2016-7-9

function [isSucc, averError] = BAbyCoreData(frameId, frameCount)
    addpath('../AlgoLib');
    global params;
    global coreData;
    if frameId > size(coreData.KFList, 1) || frameId <= 2
        disp('invalid input in PoseOptimization, please check!');
        isSucc = false;
        return;
    end
    
    if frameId - frameCount <= 0
        frameCount = frameId;
    end
    
    xyzPoints = [];
    tracks = pointTrack.empty();
    camPoses = table();
    mpExistList = [];
    fixList = [];
    baCount =0;
    for m = (frameId - frameCount+1) : frameId
        baCount = baCount+1;
        if baCount<=4
            fixList(end+1) = m;
        end
        kfTmp = coreData.KFList(m);
        kpList = kfTmp.KPList;
        for n = 1 : size(kpList, 1)
            mpId = kpList(n).mpId;
            if sum(find(mpExistList == mpId))~=0 || mpId == -1
                continue;
            end
            mpTmp = coreData.MPList(mpId);
            mpPoint = mpTmp.Posi;
            if isempty(mpPoint);
               continue; 
            end
            trackList = mpTmp.Track;
            viewIDs = [];
            points = [];
            for t = 1 : size(trackList, 1)
                fid = trackList(t).FrameId;
                if fid > frameId
                    break;
                end
                if fid < frameId - frameCount+1
                    continue;
                end
                kpid = trackList(t).KPId;
                viewIDs = [viewIDs, fid];
                points = [points; coreData.KFList(fid).KPList(kpid).posi];
            end
            if size(viewIDs,2) <=1
               continue; 
            end
            xyzPoints = [xyzPoints; mpPoint];
            mpExistList = [mpExistList; mpId];
            tracks(end + 1) = pointTrack(viewIDs, points);
        end
        ViewId = cast(m, 'uint32');
        poseMatrix = [kfTmp.Pose; 0 ,0, 0, 1]^-1; % inv of our pose
        Orientation = {poseMatrix(1 : 3, 1 : 3)'};
        Location = {poseMatrix(1 : 3, 4)'};
        poseTmp = table(ViewId, Orientation, Location);
        camPoses = [camPoses; poseTmp];
    end
    cameraParams = cameraParameters('IntrinsicMatrix', params.cameraParam');
    %averError1 = CalMulFrameReproError(frameId - frameCount,frameId)
    fixList= uint32(fixList);
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'PointsUndistorted', true, 'Verbose', ...
        false, 'MaxIterations', 20, 'FixedViewID', fixList(1), 'FixedViewID', fixList(2), 'FixedViewID', fixList(3), 'FixedViewID', fixList(4));
    averError = sum(reprojectionErrors) / size(reprojectionErrors, 1);
    
    for m = 1 : size(mpExistList, 1)
        coreData.MPList(mpExistList(m)).Posi = xyzPoints(m, :);
    end
    
    for p = 1 : frameCount
        poseMatrix = [camPoses.Orientation{p}', camPoses.Location{p}'; ...
            0 ,0, 0, 1]^-1; % inv of matlab pose
        poseIdx = camPoses.ViewId(p);
        coreData.KFList(poseIdx).Pose = poseMatrix(1 : 3, :);
    end
    %averError1 = CalMulFrameReproError(frameId - frameCount,frameId)
    isSucc = true;
end

