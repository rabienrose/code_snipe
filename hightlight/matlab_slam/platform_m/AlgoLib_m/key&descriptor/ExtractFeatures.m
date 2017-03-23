% Feature extract module which include these methods below:
% 'BRISK' 'FREAK' 'SURF' 'Block' 'Auto'
% The first three are invariant in scale and rotation, but the 'Block' are
% not, so it has not the option of 'Upright'(Rotation invariance flag). Set
% 'Upright' to true when you do not need the image descriptors to capture 
% rotation information. When you set this property to false, the 
% orientation of the features is estimated and the features are then 
% invariant to rotation. 'Auto' is based on the class of the input points 
% and implements: a cornerPoints input object corresponds to 'FREAK';  
% a SURFPoints or MSERRegions input object corresponds to 'SURF'; 
% a BRISKPoints input object corresponds to 'BRISK'; an M-by-2 input matrix 
% of [x y] coordinates corresponds to 'Block'.
% 'BRISK', 'FREAK' are binary descriptors. Binary descriptors are fast but
% less precise in terms of localization. They are not suitable for 
% classification tasks.
% Example------------------------------------------------------------------
% You can call all thess methods as,
% features = extractFeatures(image, points, 'Method', '*', 'Upright', *);
% and anothor way is,
% [features, validPoints] = extractFeatures(image, points, 'Method', '*',... 
% 'Upright', *),
% which 'validPoints' means - valid points, The function extracts 
% descriptors from a region around each interest point. If the region lies 
% outside of the image, the function cannot compute a feature descriptor 
% for that point. When the point of interest lies too close to the edge of 
% the image, the function cannot compute the feature descriptor. In this 
% case, the function ignores the point. The point is not included in the 
% valid points output. You can find details using 'help detectMSERFeatures'.
% Author: Zhongjun Dai
% Date: 2016-6-27

function [features, validPoints] = ExtractFeatures(image, points, type)
	if strcmp(type, 'BRISK')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        chacha
        if nargout == 1
            features = extractFeatures(image, points, 'Method', 'BRISK',...
                'Upright', true); % binary image | 2-D grayscale image
        elseif nargout == 2
            [features, validPoints] = extractFeatures(image, points,...
                'Method', 'BRISK', 'Upright', true); 
        end
        
    elseif strcmp(type, 'FREAK')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        
        if nargout == 1
            features = extractFeatures(image, points, 'Method', 'FREAK',...
                'Upright', true); % binary image | 2-D grayscale image
        elseif nargout == 2
            [features, validPoints] = extractFeatures(image, points,...
                'Method', 'FREAK', 'Upright', true); 
        end
        
    elseif strcmp(type, 'SURF')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        % Additional Name-Value Pair Arguments: 'SURFSize' ? 
        % Length of feature vector which can be 64 (default) | 128
        % 128: greater accuracy, lower matching speed
        if nargout == 1
            features = extractFeatures(image, points, 'Method', 'SURF',...
                'Upright', true); % binary image | 2-D grayscale image
        elseif nargout == 2
            [features, validPoints] = extractFeatures(image, points,...
                'Method', 'SURF', 'Upright', true); 
        end
        
    elseif strcmp(type, 'Block')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        
        if nargout == 1
            features = extractFeatures(image, points, 'Method', 'Block'); % binary image | 2-D grayscale image
        elseif nargout == 2
            [features, validPoints] = extractFeatures(image, points,...
                'Method', 'Block'); 
        end
    
    elseif strcmp(type, 'Auto')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        
        if nargout == 1
            features = extractFeatures(image, points); % binary image | 2-D grayscale image
        elseif nargout == 2
            [features, validPoints] = extractFeatures(image, points); 
        end
        
    end
end
