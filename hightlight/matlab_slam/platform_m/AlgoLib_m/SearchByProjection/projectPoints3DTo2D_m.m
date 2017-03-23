function [points2D, zc_inv] = projectPoints3DTo2D(K, pose3D, points3D)
    R = pose3D(:, 1:3);
    t = pose3D(:, 4);
    x3Dw = points3D;
    x3Dc = bsxfun(@plus,R * x3Dw', t);

    xc = x3Dc(1, :);
    yc = x3Dc(2, :);
    zc_inv = 1.0 ./ x3Dc(3, :);

    u = K(1, 1) .* xc .* zc_inv + K(1, 3);
    v = K(2, 2) .* yc .* zc_inv + K(2, 3);
    
    u = u';
    v = v';
    zc_inv = zc_inv';
    
    points2D = zeros(size(u, 1), 2, 'double');
    for id = 1 : size(u, 1)
        points2D(id, 1) = u(id);
        points2D(id, 2) = v(id);
    end
end