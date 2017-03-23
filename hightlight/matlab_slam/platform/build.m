%mexOpenCV -I/usr/local/include -I. -I../../cplusplus/platform -L../../cplusplus/platform -lPlatformLib -outdir bin/
strOption = '-I/usr/local/include -I. -I../../cplusplus/platform -L../../cplusplus/platform -lPlatformLib -outdir bin/'
cppList = [
    {'algorithm/extractFeatures.cpp'} 
    {'algorithm/predictPose.cpp'} 
    {'algorithm/searchByProjection.cpp'} 
    {'algorithm/optimizePose.cpp'} 
    {'algorithm/trackLocalMap.cpp'} 
    {'algorithm/decisionKeyFrame.cpp'} 
    {'algorithm/searchByEpipolar.cpp'}
    {'algorithm/triangulate.cpp'}
    %{'algorithm/cullMapPoint.cpp'}
    {'algorithm/localBA.cpp'}
    {'algorithm/MatchByOpticFlow.cpp'}    
    {'algorithm/tryInitPose.cpp'}
    {'IO/SaveVisFile.cpp'}
    {'IO/setImage.cpp'}
    {'IO/SetKPsToVisPlat.cpp'}
    {'IO/SetMPsToVisPlat.cpp'}
    {'IO/readGlobalMapFromC.cpp'}
    {'IO/writeGlobalMapToC.cpp'}
    {'IO/readGlobalMapFromCWithName.cpp'}
    {'IO/writeGlobalMapToCWithPath.cpp'}
    {'IO/SaveVisData.cpp'}
    {'IO/ClearVisData.cpp'}
    {'IO/AddActMpToVisPlat.cpp'}
    {'system/destorySLAMHandle.cpp'}
    {'system/createSLAMHandle.cpp'}
    {'system/init.cpp'}
    {'system/changeCameraParam.cpp'}
    ];
for i= 1: size(cppList,1)
    strCommond = ['mexOpenCV ' cppList{i} ' ' strOption]
    eval(strCommond);
end
build_GMap