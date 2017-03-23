function mpClickCB(src, eventdata)
    global coreDataMatrix;
    global kfListUI;
    global kpListUI;
    global mpListUI;
    global trListUI;
    ind = eventdata.Indices(1);
    mpID = src.UserData.mpIDs(ind);
    mpListUI.UserData.selected = mpID;
    kfID = kfListUI.UserData.selected;
    trCount = coreDataMatrix.mpTracksCountVec(mpID, 1);
    count=0;
    useData(trCount, 1)=0;
    trLabels(trCount, 1)={''};
    for i=1:trCount
        trframeId = coreDataMatrix.mpTrackMatrix(1, i ,mpID);
        trkpId = coreDataMatrix.mpTrackMatrix(2, i ,mpID);
        if true
            count=count+1;
            trName = {['FId: ' num2str(trframeId) '  KId: ' num2str(trkpId)]};
            trLabels(count,1) = trName;
            useData(count,1) = i;
        end
    end
    trLabels = trLabels(1: count, 1);
    useData = useData(1: count, 1);
    trListUI.Data = trLabels;
    trListUI.ColumnName = {['MP(' num2str(size(trLabels, 1)) ')']};
    trListUI.UserData.trIDs = useData;
end