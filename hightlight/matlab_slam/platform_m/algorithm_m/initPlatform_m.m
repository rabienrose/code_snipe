% Init coreData: image size, allocate space of KFList.
% addpath('../CoreData');
function initPlatform_m(imageIdxSet)
    global params;
    params.imagePaths = imageSet(params.imageDir);
    frameCount = size(imageIdxSet, 1);

    % Initialize core data structure
    global coreDataMatrix;
    coreDataMatrix = coreDataStructure;
    
    mpNum = frameCount * 1000;
    if strcmp(params.featureType, 'ORB')
        descDim = 32;
        coreDataMatrix.mpDescMatrix = cast(zeros(descDim, mpNum), 'uint8');
        coreDataMatrix.kpDescMatrix = cast(zeros(descDim, params.kpNum, frameCount), 'uint8');
    else
        descDim = 64;
        coreDataMatrix.mpDescMatrix = zeros(descDim, mpNum);
        coreDataMatrix.kpDescMatrix = zeros(descDim, params.kpNum, frameCount);
    end
    
    coreDataMatrix.mpPosiMatrix = NaN(3, mpNum);
    coreDataMatrix.mpTrackMatrix= zeros(2, frameCount, mpNum);
    coreDataMatrix.mpIsBadVec = zeros(mpNum, 1);
    coreDataMatrix.mpTracksCountVec = zeros(mpNum, 1);
    
    coreDataMatrix.frPoseMatrix = zeros(3, 4, frameCount);
    coreDataMatrix.frTypeMatrix = cell(frameCount, 1);
    coreDataMatrix.frImgIdVec = zeros(params.imagePaths.Count, 1);
    coreDataMatrix.frImgDirMatrix = params.imagePaths.ImageLocation(1, imageIdxSet)';
    
    coreDataMatrix.kpPosiMatrix = zeros(2, params.kpNum, frameCount);
    coreDataMatrix.kpMpIdMatrix =[zeros(params.kpNum, 1, frameCount), ...
                                 zeros(params.kpNum, params.mpIdLength - 1, frameCount)];
    coreDataMatrix.kpOctaveMatrix = zeros(params.kpNum, frameCount);
    coreDataMatrix.kpsCountVec = zeros(frameCount, 1);
    
    img = read(params.imagePaths, 1);
    coreDataMatrix.imgSize = [size(img, 2), size(img, 1)];
    
    coreDataMatrix.frImgIdVec(imageIdxSet) = (1 : frameCount)';
    
    %%init c_plus_plus platform.
% 	params.pInstance = createSLAMHandle();
%     init(params.pInstance);     
end


