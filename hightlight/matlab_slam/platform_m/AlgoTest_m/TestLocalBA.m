close all
clear
clc

addpath('../Configuration');
addpath('../Platform');

load('../Workplace/all.mat');

frameId     = 5;
frameCount  = 4;

[isSucc, ~] = BAbyCoreData2(frameId, frameCount);