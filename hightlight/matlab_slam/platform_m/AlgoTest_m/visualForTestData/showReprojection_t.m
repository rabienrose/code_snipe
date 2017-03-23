function showReprojection_t(varargin)
    global coreDataMatrix;
    global params;
    
    % Check input
    paraCount = size(varargin, 2);
    frameId = varargin{1};
    reprojMin = -1;
    reprojMax = 10000;
    reprojOffset = 0;
    if paraCount >= 2
        reprojMin = varargin{2};
        reprojMax = varargin{3};
    end
    if paraCount >= 4
        reprojOffset = varargin{4};
    end
    
    maxImgCount = size(coreDataMatrix.frImgIdVec, 1);
    if frameId < 1 || frameId > maxImgCount
        disp('invalid input in ShowReprojection, please check!');
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner < 1
        disp('invalid inner input in ShowReprojection, please check!');
        return;
    end
    
    % Get mpId of current frame
    mpIdCount = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner);
    nonZeroIdx = (mpIdCount ~= 0);
    if sum(nonZeroIdx) == 0
        disp('empty map points for reprojection.');
        return;
    end
    mpIdSet = coreDataMatrix.kpMpIdMatrix(nonZeroIdx, 2, currIdInner); % mpIds
    
    % Get 3D and 2D points
    if reprojOffset == 0 
        % reproject to current frame
        points3D = coreDataMatrix.mpPosiMatrix(:, mpIdSet); % mps
        checkPt3D = isnan(points3D(1, :));
        if isempty(find(checkPt3D == 0, 1))
            disp('empty map points for reprojection.');
            return; 
        end
        targetKpsPosi = coreDataMatrix.kpPosiMatrix(:, nonZeroIdx, currIdInner); % kps
        points3D(:, checkPt3D) = [];
        targetKpsPosi(:, checkPt3D) = [];
    else
        % reproject to the offset frame
       row = size(mpIdSet, 1);
       points3D = zeros(3, row);
       targetKpsPosi = zeros(2, row);
       k = 1;
       for i = 1 : row
           track = coreDataMatrix.mpTrackMatrix(:, :, mpIdSet(i));
           trackIdx = track(1, :) == (currIdInner + reprojOffset);
           if sum(trackIdx) > 0
               points3D(:, k) = coreDataMatrix.mpPosiMatrix(:, mpIdSet(i)); % mps;
               if isnan(points3D(:, k))
                   points3D(:, k) = [];
                   continue;
               end
               targetSet = track(:, trackIdx);
               targetKpsPosi(:, k) = coreDataMatrix.kpPosiMatrix(:, targetSet(2, 1), tragetSet(1, 1));
               k = k + 1;
           end
       end
       if k == 1
           disp('empty map points for reprojection.');
           return; 
       end
       points3D = points3D(:, 1 : k - 1);
       targetKpsPosi = targetKpsPosi(:, 1 : k - 1);
    end
    
    % Reproject and calculate errors
    pose = coreDataMatrix.frPoseMatrix(:, :, currIdInner + reprojOffset);
    reproPoints2D = params.cameraParam * pose * [points3D; ones(1, size(points3D, 2))];
    reproPoints2D = reproPoints2D ./ repmat(reproPoints2D(3, :), 3, 1); % reproject kps
    
    
    col = size(reproPoints2D, 2);
    errSet = zeros(col, 1);
    for i = 1 : col
       errSet(i, 1) = norm(reproPoints2D(1:2, i) - targetKpsPosi(:, i));
    end
    
    showIdx = errSet >= reprojMin & errSet <= reprojMax;
    count = size(showIdx, 1);
    if count > 0
        reproPts2DShow = reproPoints2D(:, showIdx);
        targetKpsShow = targetKpsPosi(:, showIdx);
        averErrShow = sum(errSet(showIdx)) / count;
        errorTxt = ['AverageError: ', num2str(averErrShow),...
            '  MPCount: ' ,num2str(col),...
            '  InlierCount: ' ,num2str(size(reproPts2DShow, 2))]; 

        % Show
        image = imread(coreDataMatrix.frImgDirMatrix{currIdInner});
        imshow(image);

        hold on
        plot(reproPts2DShow(1, :), reproPts2DShow(2, :), 'ro');
        plot(targetKpsShow(1, :), targetKpsShow(2, :), 'b*');
        text(1, 1, errorTxt, 'EdgeColor', 'green', 'BackgroundColor', 'white', 'FontSize', 10);
        title('Reprojection');
    end
end

