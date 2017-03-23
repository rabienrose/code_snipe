function onBtnClick(src, eventdata, x)
    global kfListUI;
    
    if x==1
        selKF = kfListUI.UserData.selected;
        global RawFeatWin;
        global OctaveFilterMin;
        global OctaveFilterMax;
        if ~isvalid(RawFeatWin)
            RawFeatWin = figure('Visible', 'off');
        end
        figure(RawFeatWin);
        octMin = str2num(OctaveFilterMin.String);
        octMax = str2num(OctaveFilterMax.String);
        showRawFeature(selKF, octMin, octMax);
        RawFeatWin.Visible ='on';
    end
    
    if x==2
        selKF = kfListUI.UserData.selected;
        global ReprojWin;
        global RepErrFilterMin;
        global RepErrFilterMax;
        global ReprojErr;
        global ReprojOffSet;
        global DepthFilterMin;
        global DepthFilterMax;
        global LifeFilterMin;
        global LifeFilterMax;
        if ~isvalid(ReprojWin)
            ReprojWin = figure('Visible', 'off');
        end
        figure(ReprojWin);
        repErrFilterMin = str2num(RepErrFilterMin.String);
        repErrFilterMax = str2num(RepErrFilterMax.String);
        depthFilterMin = str2num(DepthFilterMin.String);
        depthErrFilterMax = str2num(DepthFilterMax.String);
        lifeFilterMin = str2num(LifeFilterMin.String);
        lifeFilterMax = str2num(LifeFilterMax.String);
        reprojOffSet = str2num(ReprojOffSet.String);
        showReprojection(selKF, repErrFilterMin, repErrFilterMax, reprojOffSet, depthFilterMin, depthErrFilterMax, lifeFilterMin, lifeFilterMax);
        ReprojWin.Visible ='on';
    end
    
    if x==3
        selKF = kfListUI.UserData.selected;
        global TrackWin;
        global TrackFilterMin;
        global TrackFilterMax;
        global TrackBack;
        if ~isvalid(TrackWin)
            TrackWin = figure('Visible', 'off');
        end
        figure(TrackWin);
        trackFilterMin = str2num(TrackFilterMin.String);
        trackFilterMax = str2num(TrackFilterMax.String);
        trackBack = str2num(TrackBack.String);
        showTrack(selKF, trackFilterMin, trackFilterMax, trackBack);
        TrackWin.Visible ='on';
    end
    
    if x==4
        global handler;
        global coreDataMatrix;
        global Channel;
        selKF = kfListUI.UserData.selected;
        channel = str2num(Channel.String);
        writeGlobalMapToC(handler, coreDataMatrix);
        SetMPsToVisPlat(handler, channel, selKF);
        fileName = ['/Volumes/chamo/dataset/mps' num2str(channel) '.chamo'];
        SaveVisData(handler, fileName, channel, 1);
    end
    
    if x==5
        global MPWin;
        if ~isvalid(MPWin)
            MPWin = figure('Visible', 'off');
        end
        figure(MPWin);
        selKF = kfListUI.UserData.selected;
        showMappoints(selKF);
    end
    
    if x==6
        global chListUI;
        chListUI.Data = [];
        chListUI.ColumnName = {['KP(' num2str(0) ')']};
        chListUI.UserData.mpIDs = [];
    end
    
    if x==7
        global chListUI;
        global handler;
        global coreDataMatrix;
        global Channel;
        channel = str2num(Channel.String);
        ClearVisData(handler, channel, 7);
        mpList = chListUI.UserData.mpIDs;
        for i=1:size(mpList,1)
            posi = coreDataMatrix.mpPosiMatrix(:,mpList(i));
            AddActMpToVisPlat(handler, channel, mpList(i),posi',[255 0 0]);
        end
        fileName = ['../../../output/actMP' num2str(channel) '.chamo'];
        SaveVisData(handler, fileName, channel, 7);
    end
    
    if x==8
        global handler;
        global coreDataMatrix;
        global Channel;
        channel = str2num(Channel.String);
        writeGlobalMapToC(handler, coreDataMatrix);
        SetMPsToVisPlat(handler, channel, -1);
        SetKPsToVisPlat(handler, channel);
        SaveVisFile(handler, '../../../output/allData.chamo');
    end
    
    if x==9
        global handler;
        global coreDataMatrix;
        global MPWin;
        if ~isvalid(MPWin)
            MPWin = figure('Visible', 'off');
        end
        figure(MPWin);
        kfCount = size(coreDataMatrix.frImgIdVec, 1);
        for i=1:kfCount
            showMappoints(i);
            drawnow;
        end
    end
    
    
end
