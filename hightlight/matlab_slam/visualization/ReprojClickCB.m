function txt = ReprojClickCB(src,~)
    global coreDataMatrix;
    global chListUI;
    if ~exist('coreDataMatrix')
        return;
    end
    frameId = src.UserData(1);
    kId = src.UserData(2);
    mpId = src.UserData(3);
    if ~isempty(chListUI.UserData)
        mpList = chListUI.UserData.mpIDs;
        if ~isempty(find(mpList == mpId))
            return;
        end
    end

    mpCount = 1;
    useData(mpCount, 1)=0;
    mpLabels(mpCount, 1)={''};
    count =0;
    for i=1:mpCount
        count=count+1;
        useData(count,1) = mpId;
        mpName = {['M:' num2str(mpId) ' K:' num2str(kId)]};
        mpLabels(count,1) = mpName;
    end
    mpLabels = mpLabels(1: count, 1);
    useData = useData(1: count, 1);
    
    chListUI.Data = [chListUI.Data; mpLabels];
    chListUI.ColumnName = {['Choosed(' num2str(size(mpLabels, 1)) ')']};
    if isempty(chListUI.UserData)
        chListUI.UserData.mpIDs =[];
    end
    chListUI.UserData.mpIDs = [chListUI.UserData.mpIDs; useData];
end