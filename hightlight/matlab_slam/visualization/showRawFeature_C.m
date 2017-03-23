function showRawFeature(varargin)
    global coreDataMatrix;
    global params;
    global pGMap;
    global handler;
    
    paraCount = size(varargin, 2);
    frameId = varargin{1};

    pFrame = getFrame(pGMap, frameId);
    imgAddr = getName(pFrame);
    image = imread(getFullAddress(imgAddr));
    imshow(image);
    hold on
    kpCount = getKeyPointCount(pFrame);
    uvs(:,kpCount) = [0; 0];
    for i=1: kpCount
        pKP = getKeyPoint(pFrame, i);
        uv = getUV(pKP);
        uvs(:, i) = uv;
    end
    plot(uvs(1, :), uvs(2, :), '*b');
    title('Raw Feature');
    
 
end