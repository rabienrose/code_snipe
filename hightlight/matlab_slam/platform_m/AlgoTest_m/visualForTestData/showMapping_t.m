%Draw all the cameras and mps

function showMapping_t(showMP)
    global coreDataMatrix;
%     fig = figure(10);
%     clf(fig);
    
    % Draw cameras
    for i = 1 : coreDataMatrix.frCount
        pose = coreDataMatrix.frPoseMatrix(:, :, i);
        poseMatrix = [pose; 0 ,0, 0, 1]^-1; % inv of our pose
        R = poseMatrix(1 : 3, 1 : 3)';
        t = poseMatrix(1 : 3, 4);
        label = num2str(i);
        plotCamera('Location', t, 'Orientation', R, 'Size', 0.3, ...
            'Color', 'r', 'Opacity', 0);%, 'label', label);
        hold on
        currT = t;
    end
    
    % Draw map points
    if showMP == true
        points3DAll = coreDataMatrix.mpPosiMatrix(:, ~coreDataMatrix.mpIsBadVec(1 : coreDataMatrix.mpCount));
        [~, col] = find(points3DAll(3, :) <= 100 & ~isnan(points3DAll(3, :)));
        points3D = points3DAll(:, col);
        ptCloud = pointCloud(points3D');
        pcshow(ptCloud, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
            'MarkerSize', 45);
    end

    title('Show Mapping');
     axis equal;
%     xlim([currT(1) - 20, currT(1) + 20]);
%     ylim([-10, 10]);
%     zlim([currT(3) - 20, currT(3) + 20]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
%     view(0,0);
end
