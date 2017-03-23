function err = calcReProjectionError(K, pose3D, points3D, points2D)
    % project points to 2D
    newPoints2D = projectPoints3DTo2D(K, pose3D, points3D);
    
    % calculate re-projection error
    diff = newPoints2D - points2D;
    
    err = 0;
    for id = 1 : size(diff)
        err = err + sqrt(diff(id, 1)*diff(id, 1) + diff(id, 2)*diff(id, 2));
    end
    err = err / size(diff, 1);
end