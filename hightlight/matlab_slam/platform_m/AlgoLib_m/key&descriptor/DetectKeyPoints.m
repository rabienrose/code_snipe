% key points detect module which include these methods below:
% 'FAST' 'Harris' 'MinEigen' 'BRISK' 'SIFT' 'SURF' 'MSER'
% The first four are corner based, the next two are blob
% based, and the last one is region based. 
% Example------------------------------------------------------------------
% You can call all thess methods as,
% keyPoints = DetectFeatures(image, 'type');
% and anothor way for the last methods,
% [keyPoints, cc] = DetectFeatures(image, 'type'),
% which 'cc' means - connected component structure.
% You can find details using 'help detectMSERFeatures'.
% Author: Zhongjun Dai
% Date: 2016-6-24

function [keyPoints, mserCC] = DetectKeyPoints(image, type)
	if strcmp(type, 'FAST')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        keyPoints = detectFASTFeatures(image); % specified in 2-D grayscale
   
    elseif strcmp(type, 'Harris')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        keyPoints = detectHarrisFeatures(image); % specified in 2-D grayscale
    
    elseif strcmp(type, 'MinEigen')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        keyPoints = detectMinEigenFeatures(image); % specified in 2-D grayscale
        
    elseif strcmp(type, 'SIFT')
        
        keyPoints = detectSIFTFeatures(image); % TODO...
    
    elseif strcmp(type, 'SURF')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        keyPoints = detectSURFFeatures(image, 'NumOctaves', 8); % specified in 2-D grayscale
    
    elseif strcmp(type, 'BRISK')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        keyPoints = detectBRISKFeatures(image); % specified in 2-D grayscale
        
    elseif strcmp(type, 'MSER')
        if size(image, 3) > 1
            image = rgb2gray(image);
        end
        
        if nargout == 1
            keyPoints = detectMSERFeatures(image); % specified in grayscale
        elseif nargout == 2
            [keyPoints, mserCC] = detectMSERFeatures(image);
        end
        
    end
end
