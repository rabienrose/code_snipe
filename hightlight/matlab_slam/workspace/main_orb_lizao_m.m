close all; clear all; clc

addpath(genpath('../'))

% profile off
% profile clear
% profile on
warning off all

global coreDataMatrix;
config_lizao_m;

startFrame =  1; 
endFrame = 180;
stepSize = 1;
imageIdxSet = (startFrame : stepSize : endFrame)';
% imageIdxSet = [startFrame : 3, 4 : stepSize : endFrame]';

%%
initPlatform_m(imageIdxSet);

calcAllFeatures_m(imageIdxSet);

matchByOptiFlowCore_m(startFrame + 1*stepSize, -1);

calcRTby2Frame_m(startFrame + 1*stepSize, -1);

calcMPbyTrack_m(startFrame + 1*stepSize, -1);

% showReprojection(2);
display('start looping...');
for i = startFrame + 2 : endFrame
    i
    tracking_m(i);
%     showReprojection(i);
    mapping_m(i);
%     showMapping(false);
    
%     figure(2)
%     showReprojection(i);
%     figure(3)
%     showTrack(i);
%      drawnow;
end

coreDataMatrix.frCount = endFrame - startFrame + 1;
showMapping(true);
 
% clk = clock;
% time = strcat(num2str(clk(1)), num2str(clk(2)), num2str(clk(3)),...
%               num2str(clk(4)), num2str(clk(5)), num2str(round(clk(6))));
% profsave(profile('info'), strcat('/users/test/desktop/speeduptest/profile_results_orbMatrix1-100', time)); 
 
 
 
