errCount =0;
singleTrackCount=0;
mpkpUnmatchCount =0;
kpVoidMpCount =0;
display('checking MPList...')
for i= 1: size(coreData.MPList,1)
    trackLen = size(coreData.MPList(i).Track,1);
    for j=1: trackLen
        frameId = coreData.MPList(i).Track(j).FrameId;
        kpId = coreData.MPList(i).Track(j).KPId;
        if coreData.KFList(frameId).KPList(kpId).mpId ~= i
            mpkpUnmatchCount=mpkpUnmatchCount+1;
        end
    end
    if trackLen <=1
        singleTrackCount=singleTrackCount+1;
        continue;
    end
    for j=1: trackLen
        for k=j+1: trackLen
            if coreData.MPList(i).Track(j).FrameId == coreData.MPList(i).Track(k).FrameId
                errCount=errCount+1;
                display('duplicated!');
                break;
            end
        end
    end
end
display(['dup: ' num2str(errCount) ...
    '    singleTrack: ' num2str(singleTrackCount) ...
    '    mpUnmatch: ' num2str(mpkpUnmatchCount)]);

display('checking KFList...')
for i=1:size(coreData.KFList, 1)
    for j=1: size(coreData.KFList(i).KPList, 1)
        mpID = coreData.KFList(i).KPList(j).mpId;
        if mpID~=-1
            kpId = getMatchedPt(coreData.MPList(mpID), i);
            if kpId~= j
                kpVoidMpCount=kpVoidMpCount+1;
            end
        end
    end
end
display(['kpUnmatch: ' num2str(kpVoidMpCount)]);