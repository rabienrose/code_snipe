% Feature Extraction interface to platform
% Author: Zhongjun Dai
% Date: 2016-6-30

function [keys, descriptor] = featureExtractor_m(image, kptype, desctype)
%     addpath('../AlgoLib/Keys/ORBExtractor');
%     addpath('../AlgoLib/key&descriptor');
    
    global params;
    
    if strcmp(kptype, 'ORB') % merge detection and extraction together
        if params.ORB_initFlag == 0
            params.ORB_initFlag = 1;
            ORB_Param = [double(params.ORB_nFeatures), ...
                        double(params.ORB_scaleFactor), ...
                        double(params.ORB_nLevels), ...
                        double(params.ORB_iniThFAST), ...
                        double(params.ORB_minThFAST)];
            ExtractORB_m('init', ...
                        ORB_Param);
        end
        [errType, keys, descriptor] = ExtractORB_m(image);
        if (errType ~= 0)
            str = 'Extract ORB fiailed'
            return;
        end
    else
        keys = DetectKeyPoints(image, kptype);
        descriptor = ExtractFeatures(image, keys, desctype);
    end

end
