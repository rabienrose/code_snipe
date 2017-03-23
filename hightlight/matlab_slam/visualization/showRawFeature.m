function showRawFeature(varargin)
    global coreDataMatrix;
    global params;
    
    paraCount = size(varargin, 2);
    frameId = varargin{1};
    octMin = -1;
    octMax = 10;
    if paraCount >= 2
        octMin = varargin{2};
        octMax = varargin{3};
    end
    
    if frameId < 1
        disp('invalid input in ShowRawFeature, please check!');
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner < 1
        disp('invalid inner input in ShowRawFeature, please check!');
        return;
    end
    
    % Show
    image = imread(getFullAddress(coreDataMatrix.frImgDirMatrix{currIdInner}));
    % just for test
%     image = zeros(coreDataMatrix.imgSize(2), coreDataMatrix.imgSize(1));
    imshow(image);
    hold on
    octave = coreDataMatrix.kpOctaveMatrix(:, currIdInner);
    showIdx = octave >= octMin & octave <= octMax;
    keyPointsShow = coreDataMatrix.kpPosiMatrix(:, showIdx, currIdInner);
    plot(keyPointsShow(1, :), keyPointsShow(2, :), '*b');
    title('Raw Feature');
 
end