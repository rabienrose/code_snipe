close all
clear all
clc

addpath(genpath('../'));

global coreDataMatrix;
config_zili;
load('/Volumes/chamo/dataset/try_mat_out.mat')
handler = createSLAMHandle();
init(handler, '/Volumes/chamo/working/matlab_slam/v2.0/config/config.xml');
GMap = getGMapPtr(handler);
vis_main;