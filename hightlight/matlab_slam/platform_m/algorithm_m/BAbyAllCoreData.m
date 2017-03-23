% Execete BA using all current data in coreData

function [averError] = BAbyAllCoreData(frameId, frameCount)
    global coreData;
    global params;
    
    mpList = coreData.MPList;
    MPNum = size(mpList, 1);
    xyzPoints = zeros(MPNum, 3);
    tracks = pointTrack.empty();
    count = 1;
    mpExistId = zeros(MPNum, 1);
    for i = 1 : MPNum
        mp = mpList(i);
        mp_posi = mp.Posi;
        if isempty(mp_posi)
            continue;
        end
        
        mp_track = mp.Track;
        trackNum = size(mp_track, 1);
        viewIDs = zeros(1, trackNum);
        points = zeros(trackNum, 2);
        count1 = 1;
        for j = 1 : trackNum
            fId = mp_track(j).FrameId;
            if fId > frameId
                break;
            end
            kpId = mp_track(j).KPId;
            viewIDs(1, count1) = fId;
            points(count1, :) = coreData.KFList(fId).KPList(kpId).posi;
            count1 = count1 + 1;
        end
        viewIDs = viewIDs(:, 1 : count1 - 1);
        points = points(1 : count1 - 1, :);
        if ~isempty(viewIDs)
            xyzPoints(count, :) = mp_posi;
            tracks(count) = pointTrack(viewIDs, points);
            mpExistId(count, :) = i;
            count = count + 1;
        end
    end
    xyzPoints = xyzPoints(1 : count - 1, :);
    mpExistId = mpExistId(1 : count - 1, :);
    
    kfList = coreData.KFList;
    KFNum = frameId; %size(kfList, 1);
    camPoses = table();
    for n = 1 : KFNum
       kf = kfList(n);
       kf_pose = kf.Pose;
       poseC2W = [kf_pose; 0, 0, 0, 1]^-1;
       Orientation = {poseC2W(1 : 3, 1 : 3)'}; % conver to matlab format£¬ premultiplication
       Location = {poseC2W(1 : 3, 4)'};
       ViewId = cast(n, 'uint32');
       campose = table(ViewId, Orientation, Location);
       camPoses = [camPoses; campose];
    end
    
    cameraParams = cameraParameters('IntrinsicMatrix', params.cameraParam');
    
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints,...
        tracks, camPoses, cameraParams,...
        'PointsUndistorted', true, 'Verbose', true);

     averError = sum(reprojectionErrors) / size(reprojectionErrors, 1)
    
    for m = 1 : size(xyzPoints, 1)
        coreData.MPList(mpExistId(m)).Posi = xyzPoints(m, :);
    end
    
    for p = 1 : KFNum
        poseW2C = [camPoses.Orientation{p}', camPoses.Location{p}'; ...
            0 ,0, 0, 1]^-1; % inv of matlab pose and w2c
        frameIdx = camPoses.ViewId(p);
        coreData.KFList(frameIdx).Pose = poseW2C(1 : 3, :);
    end

end