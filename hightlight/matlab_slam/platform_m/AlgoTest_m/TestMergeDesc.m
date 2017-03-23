%% add path
addpath('../AlgoLib/MergeDescriptor');
addpath('../MianFlow');
%% gen
a(:,1)=ones(64,1);
a(:,2)=ones(64,1)*2;
a(:,3)=ones(64,1)*10;
a= uint8(a);
%% try
re = MergeDescriptor(a);
