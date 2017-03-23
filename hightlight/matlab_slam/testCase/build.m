%mexOpenCV -I/usr/local/include -I../platform -I../../cplusplus/TestCase -L../../cplusplus/TestCase -lTestCase -outdir bin/
strOption = '-I/usr/local/include -I. -I../../cplusplus/TestCase -L../../cplusplus/TestCase -lTestCase -outdir bin/'
cppList = [
    {'CreateTestCaseParser.cpp'}
    {'GetConfigDir.cpp'} 
    {'GetCamConfig.cpp'}
    {'GetFrameRange.cpp'}
    {'GetImagDir.cpp'}
    {'GetCaseNames.cpp'}
    ];
for i= 1: size(cppList,1)
    strCommond = ['mexOpenCV ' cppList{i} ' ' strOption]
    eval(strCommond);
end