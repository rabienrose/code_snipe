% TestSearchByEpipolar.m
% Created by zhaiq on 07/06/16.
% Copyright (@) 2016 ygomi. All rights reserved.
clear;
clc;

%%add working path.
addpath('../AlgoLib/Keys/ORBExtractor');
addpath('../AlgoLib/SearchByEpipolar');
addpath('../MianFlow');

global params;
config;

%%initialize ORB extractor.
ORB_Param =     [double(params.nfeatures), ...
                double(params.scalefactor), ...
                double(params.nlevel), ...
                double(params.initFASTTh), ...
                double(params.minFASTTh)];
ExtractORB('init', ...
           ORB_Param);

%%
%load image to calcualte ORB keys and descriptors.
frame1 = imread('../images/00001.jpg');
[res1, keys1] = ExtractORB(rgb2gray(frame1));
% figure(1);
% imshow(frame1); hold on;
% for id = 1 : size(keys1, 1)
%     point = keys1(id).KeyPoint;
%     plot(point(1), point(2), 'r.', 'MarkerSize',20);
% end
% pause(0.01);

frame2 = imread('../images/00002.jpg');
[res2, keys2] = ExtractORB(rgb2gray(frame2));

frame3 = imread('../images/00003.jpg');
[res3, keys3] = ExtractORB(rgb2gray(frame3));

%compute pose between frame1 and frame2.

%compute pose between frame2 and frame3.

referKeys = keys1(:, 1).KeyPoint;
referDesc = keys1.Descriptor;
referPose = [1.0, 0, 0, 0;
            0, 1.0, 0, 0;
            0, 0, 1.0, 0];

%currKeys = keys2.KeyPoint;
currDesc = keys2.Descriptor;
currPose = [1.0, 0, 0, 0;
            0, 1.0, 0, 0;
            0, 0, 1.0, 0];

currKeys = struct('Location', [], 'Scale', [], 'Metric', [], 'Misc', [], 'Orientation', [], 'Octave', []);
for id = 1 : 5
    pt = [id+1, id+0.5];
    currKeys.Location       = [currKeys.Location; single(pt(1)), single(pt(2))];
    currKeys.Scale          = [currKeys.Scale; single(2^id)];
    currKeys.Metric         = [currKeys.Metric; single(0)];
    currKeys.Misc           = [currKeys.Misc; cast(-1, 'uint32')];
    currKeys.Orientation    = [currKeys.Orientation; single(0)];
    currKeys.Octave         = [currKeys.Octave; cast(id, 'uint32')];
end

scaleFactorSet = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
%%search by epipolar
%[matchingPairs, matchingflagOfCurr, matchingflagOfRefer] = ...
keys =  SearchByEpipolar(currKeys, currDesc, currPose, referKeys, referDesc, referPose, params.cameraParam, scaleFactorSet);
    
    
    
    