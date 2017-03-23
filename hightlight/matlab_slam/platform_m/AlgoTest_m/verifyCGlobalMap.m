mpCount = coreDataMatrix.mpCount;
frCount = coreDataMatrix.frCount;
maxFrCount = size(coreDataMatrix.frImgIdVec,1);
maxTrackCount = size(coreDataMatrix.mpTrackMatrix,2);
maxKpCount = size(coreDataMatrix.kpPosiMatrix,2);
frIdMapper = coreDataMatrix.frImgIdVec;
frIdMapper1 = coreDataMatrix1.frImgIdVec;

%%%%%%%%%%%%%%%%%%%
%MapPoints
%%%%%%%%%%%%%%%%%%%
if sum((coreDataMatrix.mpTracksCountVec(1:mpCount,:) ~= coreDataMatrix1.mpTracksCountVec(1:mpCount,:))')
    display('mpTracksCountVec');
end

if sum((coreDataMatrix.mpIsBadVec(1:mpCount,:) ~= coreDataMatrix1.mpIsBadVec(1:mpCount,:))')
    display('mpIsBadVec');
end

if sum((coreDataMatrix.mpDescMatrix(:,1:mpCount) ~= coreDataMatrix1.mpDescMatrix(:,1:mpCount))')
    display('mpDescMatrix');
end

for i=1:mpCount
    if ~isnan(coreDataMatrix.mpPosiMatrix(1,i))
        if sum(coreDataMatrix.mpPosiMatrix(:,i) ~= coreDataMatrix1.mpPosiMatrix(:,i),1)
            display('mpPosiMatrix');
            break;
        end
    end
end

if sum(sum(sum(coreDataMatrix.mpTrackMatrix(:,1:maxTrackCount,1:mpCount) ~= coreDataMatrix1.mpTrackMatrix(:,1:maxTrackCount,1:mpCount),1),2),3)
    display('mpTrackMatrix');
end

%%%%%%%%%%%%%%%%%%%
%Frames
%%%%%%%%%%%%%%%%%%%
% for i=1:maxFrCount
%     id = frIdMapper(i);
%     id1 = frIdMapper1(i);
%     if id<=0 || id1<=0
%         continue;
%     end
%     if ~strcmp(coreDataMatrix.frImgDirMatrix{id}, coreDataMatrix1.frImgDirMatrix{id1})
%         display('frImgDirMatrix');
%         break;
%     end
% end
% 
% for i=1:maxFrCount
%     id = frIdMapper(i);
%     id1 = frIdMapper1(i);
%     if id<=0 || id1<=0
%         continue;
%     end
%     if ~strcmp(coreDataMatrix.frTypeMatrix{id}, coreDataMatrix1.frTypeMatrix{id1})
%         display('frTypeMatrix');
%         break;
%     end
% end

for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~sum(sum(coreDataMatrix.frPoseMatrix(:,:,id) == coreDataMatrix1.frPoseMatrix(:,:,id1),1),2)
        display('frPosMatrix');
        break;
    end
end

%%%%%%%%%%%%%%%%%%%
%KeyPoints
%%%%%%%%%%%%%%%%%%%
for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~coreDataMatrix.kpPosiMatrix(id) == coreDataMatrix1.kpPosiMatrix(id1)
        display('kpsCountVec');
        break;
    end
end

for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~sum(sum(coreDataMatrix.kpPosiMatrix(:,:,id) == coreDataMatrix1.kpPosiMatrix(:,1:maxKpCount,id1),1),2)
        display('kpPosiMatrix');
        break;
    end
end

for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~sum(sum(coreDataMatrix.kpDescMatrix(:,:,id) == coreDataMatrix1.kpDescMatrix(:,1:maxKpCount,id1),1),2)
        display('kpDescMatrix');
        break;
    end
end

for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~sum(coreDataMatrix.kpOctaveMatrix(:,id) == coreDataMatrix1.kpOctaveMatrix(1:maxKpCount,id1), 1)
        display('kpOctaveMatrix');
        break;
    end
end

for i=1:maxFrCount
    id = frIdMapper(i);
    id1 = frIdMapper1(i);
    if id<=0 || id1<=0
        continue;
    end
    if ~sum(sum(coreDataMatrix.kpMpIdMatrix(:,:,id) == coreDataMatrix1.kpMpIdMatrix(1:maxKpCount, :,id1),1),2)
        display('kpMpIdMatrix');
        break;
    end
end

