%mexOpenCV -I/usr/local/include -I. -I../../cplusplus/platform -I../../cplusplus/dataStructure -L../../cplusplus/platform -lPlatformLib -outdir bin/

strOption = '-I/usr/local/include -I. -I../../cplusplus/platform -I../../cplusplus/dataStructure -L../../cplusplus/platform -lPlatformLib -outdir bin/'
cppList = [
    {'IO/getGMapPtr.cpp'}
    {'IO/Frame/setPose.cpp'}
    {'IO/Frame/getPose.cpp'}
    {'IO/Frame/getName.cpp'}
    {'IO/Frame/setName.cpp'}
    {'IO/Frame/getFId.cpp'}
    {'IO/Frame/getFType.cpp'}
    {'IO/Frame/setFType.cpp'}
    {'IO/Frame/addMatchingPair.cpp'}
    {'IO/Frame/delMatchingPair.cpp'}
    {'IO/Frame/resetMatchingPair.cpp'}
    {'IO/Frame/addKey.cpp'}
    {'IO/Frame/getKeyPoint.cpp'}
    {'IO/Frame/getKeyPointCount.cpp'}
    {'IO/GlobalMap/addFrame.cpp'}
    {'IO/GlobalMap/addKeyFrame.cpp'}
    {'IO/GlobalMap/addMapPoint.cpp'}
    {'IO/GlobalMap/getAllFrames.cpp'}
    {'IO/GlobalMap/getAllMapPoints.cpp'}
    {'IO/GlobalMap/getFrame.cpp'}
    {'IO/GlobalMap/getInterestKeyFrames.cpp'}
    {'IO/GlobalMap/getLocalMapPoints.cpp'}
    {'IO/GlobalMap/getMapPoint.cpp'}
    {'IO/KeyPoint/getKPDesc.cpp'}
    {'IO/KeyPoint/getKPMPs.cpp'}
    {'IO/KeyPoint/getOctave.cpp'}
    {'IO/KeyPoint/getUV.cpp'}
    {'IO/KeyPoint/setKPDesc.cpp'}
    {'IO/KeyPoint/setKPMPs.cpp'}
    {'IO/KeyPoint/setOctave.cpp'}
    {'IO/KeyPoint/setUV.cpp'}
    {'IO/MapPoint/addTrack.cpp'}
    {'IO/MapPoint/getMPDescriptor.cpp'}
    {'IO/MapPoint/getMPId.cpp'}
    {'IO/MapPoint/getObservation.cpp'}
    {'IO/MapPoint/getPosition.cpp'}
    {'IO/MapPoint/isBad.cpp'}
    {'IO/MapPoint/mergeDescriptor.cpp'}
    {'IO/MapPoint/removeTrack.cpp'}
    {'IO/MapPoint/setBad.cpp'}
    {'IO/MapPoint/setMPDescriptor.cpp'}
    {'IO/MapPoint/setMPId.cpp'}
    {'IO/MapPoint/setPosition.cpp'}
    ];
for i= 1: size(cppList,1)
    strCommond = ['mexOpenCV ' cppList{i} ' ' strOption]
    eval(strCommond);
end