addpath('../workspace');
addpath('../platform/bin');
addpath('../testCase/bin');
addpath('../tools');
close all;
LoadFromMat = 1;

if LoadFromMat==1
    clear all;
    global handler;
    global coreDataMatrix;
    %============Interface=============%
    rootDir = '/Volumes/chamo/working/matlab_slam/';
    imagRoot = '/Volumes/chamo/dataset/Vehicle_SLAM_Video_Test_Set/';
    testCaseFileName = 'test_cases.xml';
    curCaseName = 'Cloverleaf_1';
    matType = 'output/refer/';
    %==================================%

    %get needed handler
    handler = createSLAMHandle();
    testHandler = CreateTestCaseParser([rootDir 'v2.0/config/' testCaseFileName], imagRoot);

    configName = GetConfigDir(testHandler, curCaseName);
    confDir = [rootDir 'v2.0/config/' configName];

    load([rootDir matType curCaseName '.mat']);
    init(handler, confDir);

    imageDir = GetImagDir(testHandler, curCaseName, -1);
    %imageDir = imageDir(1: end-6);
    configAddr = confDir;
    [fx fy cx cy] = GetCamConfig(testHandler, curCaseName);
    config_default;
else
    global coreDataMatrix;
end

f = figure('Position', [10 30 500 810]);
set(f, 'MenuBar', 'None');

count =0;
imgSize = size(coreDataMatrix.frImgIdVec,1);
kfLabels(imgSize, 1)={''};
useData(imgSize, 1) =0;
for i=1: imgSize
    tId = coreDataMatrix.frImgIdVec(i);
    if tId ==0
        continue;
    end
    if coreDataMatrix.frTypeMatrix(tId, 1) ==1
        count= count+1;
        KFName = {['Id:' num2str(i-1) '']};
        kfLabels(count,1) = KFName;
        useData(count,1) = i;
    end
end
kfLabels = kfLabels(1: count, 1);
useData = useData(1: count, 1);
global kfListUI;
kfListUI = uitable('Parent', f, 'Data', kfLabels, 'Position', [5 5 80 800]);
kfListUI.UserData.frameIDs = useData;
kfListUI.RowName = [];
kfListUI.ColumnName = {['KF(' num2str(size(kfLabels, 1)) ')']};
kfListUI.CellSelectionCallback = @kfClickCB;

global kpListUI;
kpListUI = uitable('Parent', f, 'Position', [90 5 100 800]);
kpListUI.RowName = [];
kpListUI.CellSelectionCallback = @kpClickCB;

global mpListUI;
mpListUI = uitable('Parent', f, 'Position', [195 405 100 400], 'ColumnWidth', {95});
mpListUI.RowName = [];
mpListUI.CellSelectionCallback = @mpClickCB;

global trListUI;
trListUI = uitable('Parent', f, 'Position', [300 405 100 400], 'ColumnWidth', {95});
trListUI.RowName = [];
trListUI.CellSelectionCallback = @trClickCB;

global chListUI;
chListUI = uitable('Parent', f, 'Position', [400 305 100 500], 'ColumnWidth', {95});
chListUI.RowName = [];
chListUI.CellSelectionCallback = @trClickCB;

ShowRawBtn=uicontrol('Parent',f,'Style','pushbutton','String','ShowRawKP','Position',[200 5 100 25]);
ShowRawBtn.Callback = {@onBtnClick, 1};
ShowReprojectBtn=uicontrol('Parent',f,'Style','pushbutton','String','ShowReproject','Position',[200 35 100 25]);
ShowReprojectBtn.Callback = {@onBtnClick, 2};
ShowTrackBtn=uicontrol('Parent',f,'Style','pushbutton','String','ShowTrack','Position',[200 65 100 25]);
ShowTrackBtn.Callback = {@onBtnClick, 3};
ShowMapBtn=uicontrol('Parent',f,'Style','pushbutton','String','SaveMP','Position',[200 95 100 25]);
ShowMapBtn.Callback = {@onBtnClick, 4};
ApplyFilterBtn=uicontrol('Parent',f,'Style','pushbutton','String','ShowMPs','Position',[200 125 100 25]);
ApplyFilterBtn.Callback = {@onBtnClick, 5};
ShowMPBtn=uicontrol('Parent',f,'Style','pushbutton','String','ClearChoosed','Position',[200 155 100 25]);
ShowMPBtn.Callback = {@onBtnClick, 6};
ProjMPBtn=uicontrol('Parent',f,'Style','pushbutton','String','SaveChoosed','Position',[200 185 100 25]);
ProjMPBtn.Callback = {@onBtnClick, 7};
GroundTruthBtn=uicontrol('Parent',f,'Style','pushbutton','String','SaveVis','Position',[200 215 100 25]);
GroundTruthBtn.Callback = {@onBtnClick, 8};
SetEpiRefBtn=uicontrol('Parent',f,'Style','pushbutton','String','Replay','Position',[200 245 100 25]);
SetEpiRefBtn.Callback = {@onBtnClick, 9};
ShowEpiRefBtn=uicontrol('Parent',f,'Style','pushbutton','String','ShowEpi','Position',[200 275 100 25]);

uicontrol('Parent', f, 'Style','text', 'String','Depth: ', 'Position',[300 5 50 25] );
global DepthFilterMin;
global DepthFilterMax;
DepthFilterMin=uicontrol('Parent', f,'Style','edit', 'String','-1','Position',[350 5 50 25]);
DepthFilterMax=uicontrol('Parent', f,'Style','edit', 'String','10000','Position',[400 5 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','Life: ', 'Position',[300 35 50 25] );
global LifeFilterMin;
global LifeFilterMax;
LifeFilterMin=uicontrol('Parent', f,'Style','edit', 'String','0','Position',[350 35 50 25]);
LifeFilterMax=uicontrol('Parent', f,'Style','edit', 'String','1000','Position',[400 35 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','RepErr: ', 'Position',[300 65 50 25] );
global RepErrFilterMin;
global RepErrFilterMax;
RepErrFilterMin=uicontrol('Parent', f,'Style','edit', 'String',-1,'Position',[350 65 50 25]);
RepErrFilterMax=uicontrol('Parent', f,'Style','edit', 'String',1000,'Position',[400 65 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','Octave: ', 'Position',[300 95 50 25] );
global OctaveFilterMin;
global OctaveFilterMax;
OctaveFilterMin=uicontrol('Parent', f,'Style','edit', 'String',-1,'Position',[350 95 50 25]);
OctaveFilterMax=uicontrol('Parent', f,'Style','edit', 'String',10,'Position',[400 95 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','TrackLen: ', 'Position',[300 125 50 25] );
global TrackFilterMin;
global TrackFilterMax;
TrackFilterMin=uicontrol('Parent', f,'Style','edit', 'String',-1,'Position',[350 125 50 25]);
TrackFilterMax=uicontrol('Parent', f,'Style','edit', 'String',1000,'Position',[400 125 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','TrackBack', 'Position',[300 155 50 25] );
global TrackBack;
TrackBack=uicontrol('Parent', f,'Style','edit', 'String',1,'Position',[350 155 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','Channel', 'Position',[300 185 50 25] );
global Channel;
Channel=uicontrol('Parent', f,'Style','edit', 'String',0,'Position',[350 185 50 25]);

uicontrol('Parent', f, 'Style','text', 'String','Reproj OffSet', 'Position',[300 215 50 25] );
global ReprojOffSet;
ReprojOffSet=uicontrol('Parent', f,'Style','edit', 'String',0,'Position',[350 215 50 25]);

global RawFeatWin;
global TrackWin;
global ReprojWin;
global MPWin;
RawFeatWin = figure('Visible', 'off');
TrackWin = figure('Visible', 'off');
ReprojWin = figure('Visible', 'off');
MPWin = figure('Visible', 'off');

