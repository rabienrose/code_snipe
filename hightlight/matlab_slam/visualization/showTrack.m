function showTrack(varargin)
    global coreDataMatrix;
    global params;
    
    % Check input
    paraCount = size(varargin, 2);
    frameId = varargin{1};
    trackLenMin = -1;
    trackLenMax = 10000;
    trackBack = 1;
    if paraCount >= 2
        trackLenMin = varargin{2};
        trackLenMax = varargin{3};
    end
    if paraCount >= 4
        trackBack = varargin{4};
    end
	
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    lastIdInner = -1;
    trackKFCount =0;
    for i=1:frameId-1
        tFId = coreDataMatrix.frImgIdVec(frameId - i);
        if tFId==0
            continue;
        end
        if coreDataMatrix.frTypeMatrix(tFId, 1) == 1
            trackKFCount= trackKFCount+1;
            if trackKFCount==trackBack
                lastIdInner = tFId;
                break;
            end
        end
    end
    if currIdInner < 1 && lastIdInner<1
        disp('invalid inner input in ShowTrack, please check!');
        return;
    end
    
    % Get mpId of current frame
    k=0;
    for i=1:size(coreDataMatrix.kpMpIdMatrix, 1)
        if coreDataMatrix.kpMpIdMatrix(i, 1, currIdInner) == 0
            continue;
        end
        mpid = coreDataMatrix.kpMpIdMatrix(i, 2, currIdInner);
        for j= 1: coreDataMatrix.mpTracksCountVec(mpid)
            if coreDataMatrix.frImgIdVec(coreDataMatrix.mpTrackMatrix(1, j, mpid)) == lastIdInner
                kp1 = coreDataMatrix.kpPosiMatrix(:, coreDataMatrix.mpTrackMatrix(2, j, mpid), lastIdInner);
                kp2 = coreDataMatrix.kpPosiMatrix(:, i, currIdInner);
                len = norm(kp1 - kp2);
                if len < trackLenMin || len >trackLenMax
                    continue;
                end
                k=k+1;
                matchedPoints1(:, k) = kp1;
                matchedPoints2(:, k) = kp2;
            end
        end
        
    end

    matchedPoints1 = matchedPoints1(:, 1 : k)';
    matchedPoints2 = matchedPoints2(:, 1 : k)';
    
    if size(matchedPoints1,1) == 0
        display('no track!!');
        return;
    end
    
    % Show
    image1 = imread(getFullAddress(coreDataMatrix.frImgDirMatrix{lastIdInner}));
    image2 = imread(getFullAddress(coreDataMatrix.frImgDirMatrix{currIdInner}));
    % just for test
%     image1 = zeros(coreDataMatrix.imgSize(2), coreDataMatrix.imgSize(1));
%     image2 = image1;
    showMatchedFeatures(image1, image2, matchedPoints1, matchedPoints2);
    title('Track');
    
   