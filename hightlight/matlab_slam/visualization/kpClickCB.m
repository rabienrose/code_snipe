function kpClickCB(src, eventdata)
    global coreDataMatrix;
    global kfListUI;
    global kpListUI;
    global mpListUI;
    ind = eventdata.Indices(1);
    kpID = src.UserData.kpIDs(ind);
    kpListUI.UserData.selected = kpID;
    kfID = kfListUI.UserData.selected;
    tkfID = coreDataMatrix.frImgIdVec(kfID);
    mpCount = coreDataMatrix.kpMpIdMatrix(kpID,1 ,tkfID);
    count=0;
    useData(mpCount, 1)=0;
    mpLabels(mpCount, 1)={''};
    for i=2:mpCount+1
        mpId = coreDataMatrix.kpMpIdMatrix(kpID, i ,tkfID);
        if coreDataMatrix.mpIsBadVec(mpId, 1) == 0
            count=count+1;
            mpName = {['Id:' num2str(mpId) ' Tr:' num2str(coreDataMatrix.mpTracksCountVec(mpId, 1))]};
            mpLabels(count,1) = mpName;
            useData(count,1) = mpId;
        end
    end
    mpLabels = mpLabels(1: count, 1);
    useData = useData(1: count, 1);
    mpListUI.Data = mpLabels;
    mpListUI.ColumnName = {['MP(' num2str(size(mpLabels, 1)) ')']};
    mpListUI.UserData.mpIDs = useData;
end