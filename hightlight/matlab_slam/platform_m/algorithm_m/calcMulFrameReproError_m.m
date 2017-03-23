% Calculate average reprojection error of multiframes(from 'frameStartId' to 'frameEndId')

function [averError] = CalMulFrameReproError_m(frameStartId, frameEndId)
    
    global coreData;
    global params;
    if(frameStartId <= 0 || frameEndId >= size(coreData.KFList, 1))
        disp('invalid input in ShowReprojection, please check!');
        return;
    end

    num = 0;
    for n = frameStartId : frameEndId
        keyFrame = coreData.KFList(n);
        keyPoints = keyFrame.KPList;
        for i = 1 : size(keyPoints, 1)
            mpId = keyPoints(i).mpId;
            if mpId == -1
                continue;
            end
            points3D = coreData.MPList(mpId).Posi;
            if size(points3D, 1)==0
                continue;
            end
            num = num + 1;
        end
    end

    reproKeyPt = zeros(num, 2);
    X = zeros(num, 1);
    Y = zeros(num, 1);
    count = 1;
    err = 0;
    for n = frameStartId : frameEndId
        keyFrame = coreData.KFList(n);
        keyPoints = keyFrame.KPList;
        for i = 1 : size(keyPoints, 1)
            mpId = keyPoints(i).mpId;
            if mpId == -1
                continue;
            end
            points3D = coreData.MPList(mpId).Posi;
            if size(points3D,1) == 0
                continue;
            end
            pose = keyFrame.Pose; % our pose
            p =  params.cameraParam * pose *[points3D 1]';
            p = p ./ p(3);
            reproKeyPt(i, :) = p(1:2, 1)';
            X(count, :) = keyPoints(i).posi(1, 1);
            Y(count, :) = keyPoints(i).posi(1, 2);
            count = count + 1;
            err = err + norm(p(1 : 2, 1)' - keyPoints(i).posi);
        end
    end
    averError = err / count;

end