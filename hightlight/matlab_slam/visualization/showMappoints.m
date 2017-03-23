function showMappoints(varargin)
    global coreDataMatrix;
    global params;
    
    % Check input
    paraCount = size(varargin, 2);
    frameId = varargin{1};
    
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner < 1
        return;
    end
    
    % Get mpId of current frame
    mpIdCount = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner);
    nonZeroIdx = (mpIdCount ~= 0);
    if sum(nonZeroIdx) == 0
        disp('empty map points for showMappoints.');
        return;
    end
    mpIdSet = coreDataMatrix.kpMpIdMatrix(nonZeroIdx, 2, currIdInner); % mpIds
    
    % reproject to current frame
    points3D = coreDataMatrix.mpPosiMatrix(:, mpIdSet); % mps
    checkPt3D = isnan(points3D(1, :));
    if isempty(find(checkPt3D == 0, 1))
        disp('empty map points for showMappoints.');
        return; 
    end
    targetKpsPosi = coreDataMatrix.kpPosiMatrix(:, nonZeroIdx, currIdInner); % kps
    points3D(:, checkPt3D) = [];
    targetKpsPosi(:, checkPt3D) = [];

    
    % Reproject and calculate errors
    pose = coreDataMatrix.frPoseMatrix(:, :, currIdInner);
    reproPoints2D = params.cameraParam * pose * [points3D; ones(1, size(points3D, 2))];
    reproPoints2D = reproPoints2D ./ repmat(reproPoints2D(3, :), 3, 1); % reproject kps
    
    
    col = size(reproPoints2D, 2);
    errSet = zeros(col, 1);
    for i = 1 : col
       errSet(i, 1) = norm(reproPoints2D(1:2, i) - targetKpsPosi(:, i));
    end
    
    averErrShow = sum(errSet) / size(errSet,1);
    errorTxt = ['AverageError: ', num2str(averErrShow),...
        '  MPCount: ' ,num2str(col)]; 

    % Show
    image = imread(getFullAddress(coreDataMatrix.frImgDirMatrix{currIdInner}));
    image = rgb2gray(image);
    imshow(image);

    hold on
    plot(targetKpsPosi(1, :), targetKpsPosi(2, :), 'b*');
    text(1, 1, errorTxt, 'EdgeColor', 'green', 'BackgroundColor', 'white', 'FontSize', 10);
    title('MapPoints');
end