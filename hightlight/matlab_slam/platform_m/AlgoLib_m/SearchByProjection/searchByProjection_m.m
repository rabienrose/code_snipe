function [matchedPairs, matchedFlag, mapPointsFlag] = searchByProjection_m(mapPoints, pose3D, K, imageSize, keys)
    %search matching pairs by projection.
    %@param mapPoints 3D map points and its one-to-one correspondence descriptors.
    %                 mapPoints = struct{'MapPoint', point, 'Descriptor', desc};
    %@param pose3D    Camera pose, pose3D = [R | t]
    %@param K         Camera intrinsic parameters.
    %                    [fx, 0, cx;
    %                 K = 0, fy, cy;
    %                     0,  0,  1]
    %@param imageSize Image boundary, for discarding projected points out of boundary
    %                 imageSize = [minX, minY, maxX, maxY];
    %@param keys      Key points of image and its one-to-one correspondence descrioptors
    %                 keys = struct{'KeyPoint', key, 'Descriptor', desc};
    %@param matchedPairs [output] Matched 3D-2D pairs.
    %                             matchedPairs = struct('Position3D', 3DPt, 'Point3DId', mapPointIs, 'Position2D', 2DPt, 'Point2DId', keyID, 'Quality', 'inlier');
    %@param matchedFlag  [output] Flag identify matching information of input 2d keys.
    %                             'matched' means keys has a matched map points,
    %                             'un-matched' means key has NOT a matched map points.
    %@param mapPointsFlag[output] Flag identify matching information of input 3d map points.
    %                             'matched' means map point has a matched 2d key,
    %                             'un-matched' means map point has NOT a matched 2d key.
    
%     addpath('calcDistance/');
    
    points3D = mapPoints.MapPoint;
    [points2D, zc_inv] = projectPoints3DTo2D_m(K, pose3D, points3D);
  
    minX = imageSize(1);
    minY = imageSize(2);
    maxX = imageSize(3);
    maxY = imageSize(4);

    matchedPairs = [];    
    mapPointsFlag = repmat(char('matched'), size(mapPoints.MapPoint, 1), 1);
    mapPointsFlag = cellstr(mapPointsFlag);
        
    matchedFlag = repmat(char('un-matched'), size(keys.KeyPoint, 1), 1);
    matchedFlag = cellstr(matchedFlag);
    
    for keyId = 1 : size(mapPoints.MapPoint, 1) % for each map points, project it to 2d.
        if zc_inv(keyId) < 0
            mapPointsFlag(keyId) = cellstr('un-matched');
            continue;
        end
        x = points2D(keyId, 1);
        y = points2D(keyId, 2);
        if x < minX || x > maxX || y < minY || y > maxY
            mapPointsFlag(keyId) = cellstr('un-matched');
            continue;
        end

        radius = 6;
        [indices, area] = getFeatureIndexInArea(x, y, radius, keys);

        if(isempty(indices))
            mapPointsFlag(keyId) = cellstr('un-matched');
            continue;
        end

        bestDist = 1e4;
        bestId   = -1;
        ref_desc = mapPoints.Descriptor(keyId, :); % descriptor of map points
        for id = 1 : size(indices, 1) % for each candidate
            desc = keys.Descriptor(indices(id), :); % multi dimension                        
%             dist = computeDescriptorBestDistance(ref_desc, desc); % for one key points, may be multiple descriptors extracted.
            dist = calcDistance_m(cast(ref_desc, 'uint8'), cast(desc, 'uint8'));
            
            if(dist < bestDist)
                bestDist = dist;
                bestId   = indices(id);
            end 
        end
        if(bestDist < 100)
            matched_pair = [keyId, bestId];
            matchedPairs = [matchedPairs; matched_pair];
            
            matchedFlag(bestId) = cellstr('matched');
        end
    end    
end

function [indices, area] = getFeatureIndexInArea(x, y, r, keys)
    % get keys' index according to given area   
    area = [x - r, y - r, x + r, y + r];
        
    pt_x = keys.KeyPoint(:, 1);
    pt_y = keys.KeyPoint(:, 2);

    indices = find(pt_x >= area(1) & pt_x <= area(3) & pt_y >= area(2) & pt_y <= area(4));
end

%% compute
% function bestDist = computeDescriptorBestDistance(refer_desc, query_desc)
%     %@param refer_desc Reference descriptor
%     %@param query_desc Descriptors set
% 
%     bestDist = 1e4;
%     for descId = 1 : size(query_desc)
%         dist = computeDistance(refer_desc, query_desc(descId, :));
%         if dist < bestDist
%             bestDist = dist;
%         end
%     end
% 
%     function answer = computeDistance(desc1, desc2)
%         assert(size(desc1, 2) == size(desc2, 2), 'computeDistance Error:', 'descriptors size is not same.');
%         if isa(desc1(1), 'double') || isa(desc1(1), 'single')
%             d = 0;
%             for idx = 1 : size(desc1, 2)
%                 d = d + (desc1(1,idx) - desc2(1,idx)) * (desc1(1,idx) - desc2(1,idx));
%             end
%             answer = sqrt(d);
%         else
%             answer = 0;
%             for idx = 1 : size(desc1, 2)
%                 answer = answer + disthamming(desc1(idx), desc2(idx));
%             end
%         end        
%     end
% 
%     function [ sum ] = disthamming( x,y )
%     % By: Francisco Carlos Calder??n Bocanegra
%     % disthamming Find the  hamming distance betwen two unsigned integer numbers
%     % this must be uint8, uint16, uint32 or uint64
%     % under Open source BSD licence
%     % http://www.opensource.org/licenses/bsd-license.php
% 
%         isuint8=isa(x, 'uint8'); % returns true
%         isuint16=isa(x, 'uint16'); % returns true
%         isuint32=isa(x, 'uint32'); % returns true
%         isuint64=isa(x, 'uint64'); % returns true
%         if isuint8
%             z = bitxor(x, y);
%             sum=0;
%             for i=1:8
%                 sum = sum + bitget(z,i);
%             end
%         end
%         if isuint16
%             z = bitxor(x, y);
%             sum=0;
%             for i=1:8
%                 sum = sum + bitget(z,i);
%             end
%         end
%         if isuint32
%             z = bitxor(x, y);
%             sum=0;
%             for i=1:32
%                 sum = sum + bitget(z,i);
%             end
%         end
%         if isuint64
%             z = bitxor(x, y);
%             sum=0;
%             for i=1:64
%                 sum = sum + bitget(z,i);
%             end
%         end
%     end
% end