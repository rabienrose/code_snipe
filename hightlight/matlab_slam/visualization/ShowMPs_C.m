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
    image = rgb2gray(image);
    imshow(image);
    hold on
    kpCount = getKeyPointCount(pFrame);
    uvs(:,kpCount) = [0; 0];
    count =0;
    for i=1: kpCount
        pKP = getKeyPoint(pFrame, i);
        uv = getUV(pKP);
        mpList = getKPMPs(pKP);
        if size(mpList,2)>0
            count=count+1;
            uvs(:, count) = uv;
        end
    end
    uvs = uvs(:,1:count);
    plot(uvs(1, :), uvs(2, :), '*b');
    errorTxt = ['MPCount: ' ,num2str(count)]; 
        
    text(1, 1, errorTxt, 'EdgeColor', 'green', 'BackgroundColor', 'white', 'FontSize', 10);
    title('MapPoints');
 
end