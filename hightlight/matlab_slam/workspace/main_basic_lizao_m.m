close all; clear all; clc;

addpath(genpath('../'))
% rmpath(genpath('../.git'))
% rmpath(genpath('../Thirdparty'))

% profile off
% profile clear
% profile on
warning off all

global coreDataMatrix;
config_lizao_m;

startFrame =  1;
endFrame = 150;
stepSize = 1;
imageIdxSet = (startFrame : stepSize : endFrame)';

tic
%%  coreData style
initPlatform_m(imageIdxSet);

calcAllFeatures_m(imageIdxSet);

calcAllTrackby2DMatch_m(imageIdxSet);

calcRTby2Frame_m(startFrame + 1,  - 1);

calcMPbyTrack_m(startFrame + 1, -1);



%%
%load('chamo1');
display('start looping...');
for i = startFrame + 2 : endFrame 
    i
    predictPose_m(i);
    poseOptimization_m(i);
    for j = 1 : 10
        calcMPbyTrack_m(i, -j);
    end
%     figure(1)
%     showMapEach(i)
%     hold on
 
    figure(2)
    showReprojection(i);
    drawnow;
end
toc
%%
 coreDataMatrix.frCount = endFrame - startFrame + 1;
 showMapping(true);

% clk = clock;
% time = strcat(num2str(clk(1)), num2str(clk(2)), num2str(clk(3)),...
%               num2str(clk(4)), num2str(clk(5)), num2str(round(clk(6))));
% profsave(profile('info'), strcat('/users/test/desktop/speeduptest/profile_results_basicMatrix', time));

% save tmpVar2
% load('tmpVar2.mat');

% for i = 2 : 10
%     figure(i)
%     ShowReprojection(i);
% end

% figure(1)
% for i = 2 : 7
%     ShowMapping(i);
% end

% figure(2)
% for i = 2 : 7
%     ShowMapping(i);
% end


