%% Extract Features from a  Image as a fake Descriptor of our test data 

function FakeDescriptor = ChooseFakeDescriptor(Num)
  rmpath('../../matlab/platform/bin')
%   Feature number (less than 7000)
%     Num = 2000; 
%%   Read image.
%     addpath('../images');
    I = imread('00004.jpg');   
    I = rgb2gray(I);
%     I = imread('cameraman.tif');
    
%%   Find features 
%       points = detectSURFFeatures(I);
%     points = detectMSERFeatures(I);
%     points = detectFASTFeatures(I);
%     points = detectBRISKFeatures(I);
    points = detectHarrisFeatures(I)
    % Notice: V2.0 has a same name function (extractFeatures.mexmaci64)
    %         So the path should be removed
    [features, valid_points] = extractFeatures(I,points);
    
%%   Display SURF features corresponding to the MSER ellipse centers.
%     figure; imshow(I); hold on;
%     plot(valid_points.selectStrongest(10),'showOrientation',true);
     features32 = features.Features(:, 1:32);
%%   select some descriptor and computer their correlation (choose a set that have lower cross-correlation)
 
% for i = 1 : 10
%      SelectNum = Num;
%      Indall = randperm(size(features32,1));  % 180 features
%      IndSelect = sort(Indall(1:SelectNum),'ascend');
%      FeatureChoosetemp = features32(IndSelect,:);
%      
%      Cross_Corr = corr(FeatureChoosetemp',FeatureChoosetemp');
%      
%      Sum(i) = sum(sum(Cross_Corr));
%      FeatureAll{i} = FeatureChoosetemp;    
%  end
%      [row,cols,~] = find(Sum == min(Sum));
%      min(Sum);
%      % choose a set that have minimal cross-correlation
%      FeatureChoose = FeatureAll{1,cols};
     SelectNum = Num;
     Indall = randperm(size(features32,1));  
     IndSelect = sort(Indall(1:SelectNum),'ascend');
     FeatureChoosetemp = features32(IndSelect,:);
     FakeDescriptor = uint8(FeatureChoosetemp);
end