mexOpenCV -Iinclude -I/usr/local/include -I/usr/local/Cellar/eigen/3.2.8/include/eigen3 -L/usr/local/lib -Llib -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_features2d -lg2o -lDBoW2 CallTracker.cpp source/Tracking.cc source/Converter.cc source/Frame.cc source/Initializer.cc source/KeyFrame.cc source/KeyFrameDatabase.cc source/Map.cc source/MapPoint.cc source/Optimizer.cc source/ORBextractor.cc source/ORBmatcher.cc source/PnPsolver.cc source/Sim3Solver.cc source/System.cc -outdir bin