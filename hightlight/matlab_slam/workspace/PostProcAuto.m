
clear all;
addpath('../workspace');
addpath('../platform/bin');
addpath('../testCase/bin');

confRootDir = '/Users/lab8admin/Documents/SLAM_AUTO_TEST/code/v2.0/config/';
confDir = [confRootDir 'test_cases.xml'];
imagRoot = '';
curCaseName = '';
testHandler = CreateTestCaseParser(confDir,imagRoot);
caseNames = GetCaseNames(testHandler);
handler = createSLAMHandle();
init(handler, '/Users/lab8admin/Documents/SLAM_AUTO_TEST/code/v2.0/config/config.xml');

caseCount = size(caseNames,1);
global coreDataMatrix;
for i=1:caseCount
    testName=['../../../output/auto_run/' caseNames{i} '.mat'];
    if ~exist(testName, 'file')
        continue;
    end
    load(testName);
    writeGlobalMapToC(handler, coreDataMatrix);
    SetKPsToVisPlat(handler, i*10);
    %SetMPsToVisPlat(handler, i*10+1, -1); 
    referName=['../../../output/refer/' caseNames{i} '.mat'];
    if ~exist(testName, 'file')
        continue;
    end
    load(referName);
    writeGlobalMapToC(handler, coreDataMatrix);
    SetKPsToVisPlat(handler, i*10+1);
    %SetMPsToVisPlat(handler, i*10, -1);
     
end
SaveVisFile(handler, '../../../output/visCompareAll.chamo');