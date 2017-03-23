% TestExtractORB.m
% Created by zhaiq on 07/01/16.
% Copyright (@) 2016 ygomi. All rights reserved.

close all;
clear;
clc;

%add path
addpath('../AlgoLib/Keys/ORBExtractor');
addpath('../MianFlow');

global params;
config;

do_initialization = true;
if do_initialization
    %init extractor
    
    ORB_Param = [double(params.nfeatures), ...
                double(params.scalefactor), ...
                double(params.nlevel), ...
                double(params.initFASTTh), ...
                double(params.minFASTTh)];
    ExtractORB('init', ...
                ORB_Param);
end

%load images
imageDir = '/Users/zhaiq/Downloads/images';
imagePaths = imageSet(imageDir);

%extract keys
for frameId = 2 : imagePaths.Count
    path = imagePaths.ImageLocation{frameId};
    frame = imread(path);
    
    if frameId < 100
        % normal correct calling
        [res, keys, desc] = ExtractORB(rgb2gray(frame));
        
        % show keys
        figure(1);
        imshow(frame);
        hold on;
        for id = 1 : size(keys, 1)
            point = keys(id).KeyPoint;
            plot(point(1), point(2), 'r.', 'MarkerSize',20);
        end
        pause(0.01);
%     elseif frameId < 15
%         % normal correct calling
%         [keys] = ExtractORB(path);
%     elseif frameId < 20
%         % input parameters error
%         [res, keys] = ExtractORB();            
%     elseif frameId < 25
%         % output parameters error
%         ExtractORB(path);
%     elseif frameId < 30
%         ExtractORB();
    end
    
end