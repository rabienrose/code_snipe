%%  build all mex files of Version 1.0

strOption = [
    {'-I./AlgoLib_m/ -I/usr/local/include -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_highgui -outdir bin_m/'}
    {'-I./AlgoLib_m/ -I../../ -I../OptimizePose/Converter.cc -I./AlgoLib_m/OptimizePose/ -I/usr/local/include  -I/usr/local/Cellar/eigen/3.2.8/include/eigen3 -L../../3rdparty/g2o -L/usr/local/lib -lg2o -lopencv_core -outdir bin_m/'}
    {'-I/usr/local/include -L/usr/local/lib -lopencv_core -outdir bin_m/'},
    {'-I./AlgoLib_m/OptiFlow/ -I./AlgoLib_m/OptiFlow/kd_tree -I/usr/local/include -L/usr/local/lib -lopencv_core -outdir bin_m/'},
    {'-I./AlgoLib_m/ -I../../  -I/usr/local/include  -I/usr/local/Cellar/eigen/3.2.8/include/eigen3 -I../ -L/usr/local/lib -L../../3rdparty/g2o  -lg2o -lopencv_core -outdir bin_m/'},
    {'-I./AlgoLib_m/ -I/usr/local/include -I../ -L/usr/local/lib -lopencv_core -outdir bin_m/'},
    {'-I./AlgoLib_m/ -I/usr/local/include -I../../ -L/usr/local/lib -lopencv_core -outdir bin_m/'},
    {'-I./AlgoLib_m/ -I/usr/local/include -I.. -L/usr/local/lib -lopencv_core -outdir bin_m'},
];


cppList = [
    {'AlgoLib_m/Keys/ORBExtractor/ExtractORB_m.cpp   AlgoLib_m/Keys/ORBExtractor/ORBextractor.cc'},
    {'AlgoLib_m/LocalBA/LocalBA_m.cpp AlgoLib_m/LocalBA/utils.cpp AlgoLib_m/LocalBA/LocalBAHandle.cpp AlgoLib_m/OptimizePose/Converter.cc'}
    {'AlgoLib_m/MergeDescriptor/MergeDescriptor_m.cpp'},
    {'AlgoLib_m/OptiFlow/OptiFlow_m.cpp   AlgoLib_m/OptiFlow/optflowIni.cpp  AlgoLib_m/OptiFlow/kd_tree.cpp   AlgoLib_m/OptiFlow/rt.cpp  AlgoLib_m/OptiFlow/algo_common.cpp'},
    {'AlgoLib_m/OptimizePose/OptimizePose_m.cpp   AlgoLib_m/OptimizePose/Optimizer.cc     AlgoLib_m/OptimizePose/Converter.cc'},
    {'AlgoLib_m/SearchByEpipolar/SearchByEpipolar_m.cpp  AlgoLib_m/SearchByEpipolar/SeacrhByEpipolarHandle.cpp'},
    {'AlgoLib_m/SearchByProjection/calcDistance/CalcDistance_m.cpp'},
    {'AlgoLib_m/Triangulate/Triangulate_m.cpp   AlgoLib_m/Triangulate/TriangulateHandle.cpp'},       
];

for i= 1: size(cppList,1)
    strCommond = ['mexOpenCV ' cppList{i} ' ' strOption{i}]
    eval(strCommond);
end