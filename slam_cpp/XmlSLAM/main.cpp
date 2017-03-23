#include "XmlSLAM.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <sstream>
#include <iomanip>

int main(){
    chamo::XmlSLAM xmlObj;
    xmlObj.SaveData("/Volumes/chamo/working/matlab_slam/v2.0/cplusplus/XmlSLAM/outTry.xml");
    xmlObj.ReadData("/Volumes/chamo/working/matlab_slam/v2.0/cplusplus/XmlSLAM/outTry.xml");
    return 0;
}
