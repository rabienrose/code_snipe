function showMapEach_t(frameId)
    global coreDataMatrix;
    global Dataparams;
    
    if frameId <= 1 || frameId > Dataparams.imagePaths.Count
        disp('invalid input in ShowMapEach, please check!');
        return;
    end
    currIdInner = coreDataMatrix.frImgIdVec(frameId);
    if currIdInner <= 1
        disp('invalid inner input in ShowMapEach, please check!');
        return;
    end
    
    % Visualize the camera locations and orientations
    cameraSize = 0.3;
    plotCamera('Size', cameraSize, 'Color', 'r',  'Opacity', 0); %'Label', '1',;
    
    pose = coreDataMatrix.frPoseMatrix(:, :, currIdInner);
    poseMatrix = [pose; 0 ,0, 0, 1]^-1; % inv of our pose
    R = poseMatrix(1 : 3, 1 : 3)';
    t = poseMatrix(1 : 3, 4);
    lable = num2str(frameId);
    plotCamera('Location', t, 'Orientation', R, 'Size', cameraSize, ...
        'Color', 'r',  'Opacity', 0, 'Label', lable);
    hold on
    
    % Visualize the point cloud
    points3D = get3DPoints(currIdInner);
    ptCloud = pointCloud(points3D');
    pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 60);
    
    title('Show Map of Current Frame');    
    xlabel(' x-axis ');
    ylabel(' y-axis ');
    zlabel(' z-axis ');
%     view(0,0);
end

function [points3D] = get3DPoints(currIdInner)
    global coreDataMatrix;   
    mpIdVec1 = coreDataMatrix.kpMpIdMatrix(:, 1, currIdInner) ~= 0;
    mpsPosi = coreDataMatrix.mpPosiMatrix(:, mpIdVec1);
    mpsChoose = (mpsPosi(3, :) ~= -1 & mpsPosi(1, :) <= 1000 & ~isnan(mpsPosi(1, :)));
    points3D = mpsPosi(:, mpsChoose);
end


