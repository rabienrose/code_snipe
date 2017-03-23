global params;
configAddr = configname;
config_parseXml;
%params.imageDir = '/Volumes/chamo/dataset/KITTI/00/image_0/';
params.imageDir = imageSet{i};
params.featureType = 'ORB';

params.ORB_nFeatures =  1500;