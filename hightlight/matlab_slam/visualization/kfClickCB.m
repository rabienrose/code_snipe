function kfClickCB(src, eventdata)
    global coreDataMatrix;
    global kfListUI;    
    global kpListUI;
    ind = eventdata.Indices(1);
    kfID = src.UserData.frameIDs(ind);
    tkfID = coreDataMatrix.frImgIdVec(kfID);
    kfListUI.UserData.selected = kfID;
    kpCount = coreDataMatrix.kpsCountVec(tkfID);
    count=0;
    useData(kpCount, 1)=0;
    kpLabels(kpCount, 1)={''};
    for i=1:kpCount
        if coreDataMatrix.kpMpIdMatrix(i,1 ,tkfID) ~= 0
            count=count+1;
            useData(count,1) = i;
            kpName = {['Id:' num2str(i) ' Du:' num2str(coreDataMatrix.kpMpIdMatrix(i,1 ,tkfID))]};
            kpLabels(count,1) = kpName;
        end
    end
    kpLabels = kpLabels(1: count, 1);
    useData = useData(1: count, 1);
    kpListUI.Data = kpLabels;
    kpListUI.ColumnName = {['KP(' num2str(size(kpLabels, 1)) ')']};
    kpListUI.UserData.kpIDs = useData;
    
end