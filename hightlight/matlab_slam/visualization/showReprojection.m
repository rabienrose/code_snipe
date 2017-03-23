function showReprojection(varargin)
    global coreDataMatrix;
    global params;
    
    % Check input
    paraCount = size(varargin, 2);
    frameId = varargin{1};
    reprojMin = -1;
    reprojMax = 10000;
    depthMin = -1;
    depthMax = 10000;
    lifeMin = 0;
    lifeMax = 1000;
    reprojOffset = 0;
    if paraCount >= 2
        reprojMin = varargin{2};
        reprojMax = varargin{3};
    end
    if paraCount >= 4
        reprojOffset = varargin{4};
    end
    
    if paraCount >= 6
        depthMin = varargin{5};
        depthMax = varargin{6};
    end
    
    if paraCount >= 8
        lifeMin = varargin{7};
        lifeMax = varargin{8};
    end
    
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    lastIdInner = currIdInner;
    lastId = frameId;
    trackKFCount =0;
    for i=1:frameId-1
        tFId = coreDataMatrix.frImgIdVec(frameId - i);
        if tFId==0
            continue;
        end
        if coreDataMatrix.frTypeMatrix(tFId, 1) == 1
            trackKFCount= trackKFCount+1;
            if trackKFCount==-reprojOffset
                lastIdInner = tFId;
                lastId = frameId - i;
                break;
            end
        end
    end
    if currIdInner < 1 && lastIdInner<1
        disp('invalid inner input in ShowTrack, please check!');
        return;
    end
    
    image = imread(getFullAddress(coreDataMatrix.frImgDirMatrix{lastIdInner}));
    image = rgb2gray(image);
    im = imshow(image);
    im.PickableParts = 'none';
    hold on;
    dcm_obj = datacursormode(gcf);
    set(dcm_obj,'UpdateFcn',{@ReprojTipCB,2});    
    mpCount =0;
    inlierCount = 0;
    errT = 0;
    kpCount = size(coreDataMatrix.kpMpIdMatrix, 1);
    for i=1: kpCount
        kpmpCount = coreDataMatrix.kpMpIdMatrix(i,1,currIdInner);
        if(kpmpCount>0)
            for j=1:kpmpCount
                mpId = coreDataMatrix.kpMpIdMatrix(i,j+1,currIdInner);
                if coreDataMatrix.mpIsBadVec(mpId) ==1
                    continue;
                end
                trackLen = coreDataMatrix.mpTracksCountVec(mpId,1);
                if trackLen>=lifeMin && trackLen<=lifeMax
                    for k=1:trackLen
                        if coreDataMatrix.mpTrackMatrix(1,k,mpId) == lastId
                            mpPosi = coreDataMatrix.mpPosiMatrix(:, mpId);
                            tarPose = coreDataMatrix.frPoseMatrix(:,:,lastIdInner);
                            uv = tarPose * [mpPosi; 1];
                            depth = uv(3);
                            if depth<depthMin || depth>depthMax
                                continue;
                            end
                            uv = params.cameraParam * uv;
                            uv = uv./uv(3);
                            tarpt2 = coreDataMatrix.mpTrackMatrix(2,k,mpId);
                            pt2Posi = coreDataMatrix.kpPosiMatrix(:,tarpt2,lastIdInner);
                            err = norm(uv(1:2) - pt2Posi);
                            errT = errT+err;
                            mpCount=mpCount+1;
                            if err>reprojMin && err<reprojMax
                                inlierCount=inlierCount+1;
                                plot(uv(1), uv(2), 'ro', 'HitTest', 'off');
                                a = plot(pt2Posi(1), pt2Posi(2), 'b*', 'ButtonDownFcn',@ReprojClickCB);
                                a.UserData = [lastId tarpt2 mpId depth];
                            end
                        end
                    end
                end
                
            end
        end
    end
    errT = errT/mpCount;
    errorTxt = ['AverageError: ', num2str(errT),...
            '  MPCount: ' ,num2str(mpCount),...
            '  InlierCount: ' ,num2str(inlierCount)]; 
        
    text(1, 1, errorTxt, 'EdgeColor', 'green', 'BackgroundColor', 'white', 'FontSize', 10);
    title('Reprojection');
end

