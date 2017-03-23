function [isSucc, averError] = BAbyCoreData2(frameId, frameCount)
    % addpath('../AlgoLib');
    % addpath('../AlgoLib/LocalBA');
    global params;
    global coreData;
    
%     save all;
    
    if frameId > size(coreData.KFList, 1) || frameId <= 2
        disp('invalid input in PoseOptimization, please check!');
        isSucc = false;
        return;
    end
    
    if frameId - frameCount <= 0
        frameCount = frameId;
    end

    %% prepare data to optimize.
    frame   = struct('frameId', [], 'pose', [], 'keys', []);
    key     = struct('mpId', [], 'pt', [], 'octave', []);
    point3d = struct('id', [], 'pt', [], 'tracks', []);
    track   = struct('frameId', [], 'keyId', []);
    
    mpExistSet      = [];
    mapPointsSet    = [];
    keyFramesSet    = [];
    for id = frameId - frameCount + 1 : frameId
        frame.frameId   = [];
        frame.pose      = [];
        frame.keys      = [];
        
        currFrame    = coreData.KFList(id);
        currKPList   = currFrame.KPList;
        for n = 1 : size(currKPList, 1)
            mpId = currKPList(n).mpId;
            
            key.mpId        = mpId;
            key.pt          = currKPList(n).posi;
            key.octave      = currKPList(n).octave;
            if(isempty(frame.keys))
                frame.keys = key;
            else
                frame.keys(end+1) = key;
            end
            
            if(sum(find(mpExistSet == mpId)) ~= 0 ||... %map point exist, multi-frames
               mpId == -1) % current key point has none map point
                continue;
            end
            
            currMapPoint = coreData.MPList(mpId);
            if(isempty(currMapPoint.Posi)) % track was found but trangulation was failed.
                continue;
            end
            
            point3d.id      = [];
            point3d.pt      = [];
            point3d.tracks  = [];
            
            %update tracks of current map points.
            originTracks    = currMapPoint.Track;
            workingTracks   = [];
            for it = 1 : size(originTracks, 1)
                currTrack = originTracks(it);
%                 if (currTrack.FrameId < frameId - frameCount + 1)
%                     continue;
%                 end
%                 if(currTrack.FrameId > frameId)
%                     break;
%                 end
                
                track.frameId   = currTrack.FrameId;
                track.keyId     = currTrack.KPId;
                if(isempty(workingTracks))
                    workingTracks = track;
                else
                    workingTracks(end + 1) = track;
                end
            end
            currMapPoint.Track = workingTracks';
            point3d.id         = mpId;
            point3d.pt         = currMapPoint.Posi;
            point3d.tracks     = currMapPoint.Track;
            
            if(isempty(mapPointsSet))
                mapPointsSet        = point3d;
            else
                mapPointsSet(end+1) = point3d;
            end
            mpExistSet(end+1)       = mpId;
        end
        
        %update key frames.
        currFrame.Image     = [];
        
        frame.frameId = id;
        frame.pose = currFrame.Pose;
        frame.keys = frame.keys';
        if(isempty(keyFramesSet))
            keyFramesSet        = frame;  
        else
            keyFramesSet(end+1) = frame;
        end

    end
    mapPointsSet = mapPointsSet';
    keyFramesSet = keyFramesSet';
        
    %% try local BA.
    [updatedKeyFrames, updatedMapPoints] = LocalBA(keyFramesSet, mapPointsSet, params.cameraParam, params.scaleFactorSet);
    
    %% save optimized data to CoreData.
    % update key frames.
    for index=1 : size(updatedKeyFrames, 1)
        id = updatedKeyFrames(index).frameId;
        coreData.KFList(id).Pose = updatedKeyFrames(index).pose;
        for j=1 : size(updatedKeyFrames(index).keys, 1)
            coreData.KFList(id).KPList(j).mpId = updatedKeyFrames(index).keys(j).mpId;
        end
    end
    
    % update map points.
    err = 0;
    trackItem = TrackItem;
    for index=1 : size(updatedMapPoints, 1)
        id = updatedMapPoints(index).id;

        coreData.MPList(id).Posi  = updatedMapPoints(index).pt';
        coreData.MPList(id).Track = [];
        for j=1 : size(updatedMapPoints(index).tracks, 1)
            trackItem.FrameId = updatedMapPoints(index).tracks(j).frameId;
            trackItem.KPId    = updatedMapPoints(index).tracks(j).keyId;
            if(isempty(coreData.MPList(id).Track))
                coreData.MPList(id).Track           = trackItem;
            else
                coreData.MPList(id).Track(end + 1)  = trackItem;
            end
        end
        coreData.MPList(id).Track = coreData.MPList(id).Track';
    end
        
    isSucc      = true;
    averError   = 1.0;
end