#include "nurbs_load_api.h"
#include <iostream>
#include <string>
#include "jsoncpp.hpp"
#include "opencv2/opencv.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>

#define Json_Key_ControlPoints  "ControlPoints"
#define Json_Key_Knots          "Knots"
#define Json_Key_PaintEndPoints "PaintEndPoints"

bool gDebug = false;

    enum class NURBS_LINE_TYPE_E : uint8_t
    {
        NURBS_LINE_TYPE_SOLID_E,
        NURBS_LINE_TYPE_DASHED_E
    };
    
    struct NurbsCurveParam
    {
        NurbsCurveParam(const NURBS_LINE_TYPE_E lineType = NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_SOLID_E,
                        const double length = 0,
                        const std::vector<cv::Point3f> &ctrlPoints = std::vector<cv::Point3f>(),
                        const std::vector<double> &knots = std::vector<double>(),
                        const std::vector<double> &endPoints = std::vector<double>())
         : lineType(lineType),
           length(length),
           ctrlPoints(ctrlPoints),
           knots(knots),
           endPoints(endPoints)
        {
        }

        NURBS_LINE_TYPE_E             lineType;     // Type of the curve
        double                        length;       // The length of the curve
        std::vector<cv::Point3f>  ctrlPoints;   // Control points
        std::vector<double>           knots;
        std::vector<double>           endPoints;    // The end points of the segments, for dashed curve only
    };

//This function is called frequently
bool basisFun(int32_t i, const double u, const int32_t p, const std::vector<double> &U, std::vector<double> &N)
{
    std::vector<double> left(p+1, 0);
    std::vector<double> right(p+1, 0);

    N.clear();
    N.resize(p+1, 0);

    N[0] = 1;

    double saved, tmp;

    for (int32_t j = 1; j <= p; j++)
    {
        left[j] = u - U[i+1-j];
        right[j] = U[i+j] - u;
        saved = 0.0;

        for (int32_t r = 0; r < j; r++)
        {
            tmp = N[r] / (right[r+1] + left[j-r]);
            N[r] = saved + right[r+1] * tmp;
            saved = left[j-r] * tmp;
        }

        N[j] = saved;
    }

    if (int32_t(N.size()) == p+1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool findspan(int32_t n, int32_t p, double u, const std::vector<double> &knot, int32_t &s)
{
    if(n >= int32_t(knot.size()) || p > n)
    {
        return false;
    }

    if(u < knot[p])
    {
        s = p;
        return true;
    }
    else if(u > knot[n])
    {
        s = n;
        return true;
    }

    int32_t low = p;
    int32_t high = n+1;
    int32_t mid = (low+high)/2;

    while(1)
    {
        if (u < knot[mid])
        {
            high = mid;
        }
        else if(u >= knot[mid+1])
        {
            low = mid;
        }
        else
        {
            break;
        }
        mid = (low+high) / 2;
    }
    s = mid;

    return true;
}

bool outSample(const NurbsCurveParam &NURBSparam,
                      const std::vector<double> &U,
                      std::vector<cv::Point3f> &outPoints)
{
    if(NURBSparam.ctrlPoints.empty() || U.empty())
    {
        return false;
    }

    int32_t degree = 3 - 1;

    outPoints.clear();
    outPoints.reserve(U.size());

    int32_t numPoints = U.size();
    int32_t nCP = NURBSparam.ctrlPoints.size();

    std::vector<double> N;
    int32_t tmpInd;
    int32_t s = degree;

    cv::Point3f tmpPoint;
    for (int32_t col = 0; col < numPoints; col++)
    {
        if(!findspan(nCP-1, s, U[col], NURBSparam.knots, s))
        {
            return false;
        }

        if(!basisFun(s, U[col], degree, NURBSparam.knots, N))
        {
            return false;
        }

        tmpInd = s-degree;

        tmpPoint.x = 0;
        tmpPoint.y = 0;
        tmpPoint.z = 0;

        for(int32_t i = 0; i <= degree; i++)
        {
            tmpPoint.x += N[i] * NURBSparam.ctrlPoints[tmpInd+i].x;
            tmpPoint.y += N[i] * NURBSparam.ctrlPoints[tmpInd+i].y;
            tmpPoint.z += N[i] * NURBSparam.ctrlPoints[tmpInd+i].z;
        }

        outPoints.emplace_back(tmpPoint);
    }

    return true;
}

bool generateCurve(const NurbsCurveParam &NURBSParam,
                          const double step,
                          std::vector<cv::Point3f> &outputPoints)
{
    if(NURBSParam.ctrlPoints.empty() || step <= 0 || step > NURBSParam.length/3)
    {
        return false;
    }

    int32_t outputPointNum = ceil(NURBSParam.length/step);

    // Safety guard
    if (1 == outputPointNum)
    {
        return false;
    }

    std::vector<double> U;
    U.reserve(outputPointNum);

    if(NURBSParam.lineType == NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_SOLID_E)
    {
        double step = 1.0/(outputPointNum-1);
        double tmp = 0;

        for(int32_t i = 0; i < outputPointNum; ++i)
        {
            if (tmp>NURBSParam.endPoints[1]){
                break;
            }
            U.emplace_back(tmp);
            tmp += step;
        }
    }
    else if(NURBSParam.lineType == NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_DASHED_E)
    {
        if(NURBSParam.endPoints.size()<=2){
            std::cout<<"error: dash end point not enough!"<<std::endl;
            return false;
        }
        else if(NURBSParam.endPoints.size() % 2 != 0){
            std::cout<<"error: dash end point odd number!"<<std::endl;
            return false;
        }
        for (int32_t i = 0; i < NURBSParam.endPoints.size(); ++i)
        {
            U.emplace_back(NURBSParam.endPoints[i]);
        }
    }
    else
    {
        std::cout<<"error: line type unknown!"<<std::endl;
        return false;
    }

    if(!outSample(NURBSParam, U, outputPoints))
    {
        std::cout<<"error: line equation failed!"<<std::endl;
        return false;
    }

    return true;
}


void Conv_Json2StdVec(const Json::Value& array, std::vector<std::string>& elems)
{
    if (Json::arrayValue == array.type())
    {
        for (std::size_t i = 0; i < array.size(); i++)
        {
            elems.push_back(array[static_cast<int32_t>(i)].asString());
        }
    }
    else
    {
    }
}

bool Conv_FormatNURBS(const std::string& sNurbs, pcl::PointCloud<pcl::PointXYZRGB> &cloud, bool &isDash)
{
    try
    {
        Json::Value root;
        std::istringstream iss(sNurbs.c_str());
        iss>>root;
        
        Json::Value jsControlPoints = root[Json_Key_ControlPoints];
        Json::Value jsKnots = root[Json_Key_Knots];
        Json::Value jsPaintEndPoints = root[Json_Key_PaintEndPoints];

        std::vector<std::string> vecControlPoints, vecKnots, vecPaintEndPoints;
        Conv_Json2StdVec(jsControlPoints, vecControlPoints);
        Conv_Json2StdVec(jsKnots, vecKnots);
        Conv_Json2StdVec(jsPaintEndPoints, vecPaintEndPoints);
        
        std::vector<cv::Point3f> ctrlPoints;
        std::vector<double> knots;
        std::vector<double> endPoints;
        for (int i=0;i<vecControlPoints.size();i++){
            std::stringstream ss(vecControlPoints[i]);
            cv::Point3f pt;
            std::string num_str; 
            std::getline(ss, num_str, ',');
            pt.x = atof(num_str.c_str());
            std::getline(ss, num_str, ',');
            pt.y = atof(num_str.c_str());
            std::getline(ss, num_str, ',');
            pt.z = atof(num_str.c_str());
            ctrlPoints.push_back(pt);
        }
        for (int i=0;i<vecKnots.size();i++){
            std::stringstream ss(vecKnots[i]);
            double num;
            std::string num_str; 
            std::getline(ss, num_str, ',');
            num = atof(num_str.c_str());
            knots.push_back(num);
        }
        for (int i=0;i<vecPaintEndPoints.size();i++){
            std::stringstream ss(vecPaintEndPoints[i]);
            double num;
            std::string num_str; 
            std::getline(ss, num_str, ',');
            num = atof(num_str.c_str());
            endPoints.push_back(num);
            std::getline(ss, num_str, ',');
            num = atof(num_str.c_str());
            endPoints.push_back(num);
        }
        NURBS_LINE_TYPE_E type=NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_SOLID_E;
        isDash =false;
        if (endPoints.size()>2){
            type = NURBS_LINE_TYPE_E::NURBS_LINE_TYPE_DASHED_E;
            isDash= true;
        }
        float length=0;
        for (int i=0; i<ctrlPoints.size()-1;i++){
            cv::Point3f t_pt=ctrlPoints[i]-ctrlPoints[i+1];
            length += std::sqrt(t_pt.x*t_pt.x + t_pt.y*t_pt.y + t_pt.z*t_pt.z);
        }
        static int temp_count=0;
        temp_count++;
        //std::cout<<temp_count<<":"<<temp_count%255<<std::endl;
        NurbsCurveParam curveParam(type, length, ctrlPoints, knots, endPoints);
        std::vector<cv::Point3f> outputPoints;
        generateCurve(curveParam, 10, outputPoints);
        
        
        for (int i=0; i<outputPoints.size();i++){
            pcl::PointXYZRGB point_c;
            point_c.x=outputPoints[i].x;
            point_c.y=outputPoints[i].y;
            point_c.z=outputPoints[i].z;
            if (isDash){
                point_c.r = 255;
                point_c.g = 0;
                point_c.b = 255;
                point_c.a = 255;
            }else{
                point_c.r = 0;
                point_c.g = 255;
                point_c.b = 255;
                point_c.a = 255;
            }
            cloud.push_back(point_c);
        }
    }
    catch(std::exception& ex)
    {
        return false;
    }
}

std::ostream &operator<<(std::ostream &os, const NS_NURBS::SegmentNodeT& data)
{
    os  << "SegmentNodeT{iSegmentId|"<<data.iSegmentId
        << "|sReferencePoint|"<<data.sReferencePoint
        << "|equationDesc|{";
    for (auto it = data.equationDesc.begin(); it != data.equationDesc.end(); it++)
    {
        os << "< "<<it->first <<" , "<< it->second << " >";
    }
    os <<"} }";
    return os;
}

void Test_LoadNurbs(const NS_NURBS::ConfT& conf)
{
    //Load
    {
        if (!NS_NURBS::NurbsLoader::Instance().Load(conf))
        {
            std::cout << "Load fail" << std::endl;
            return;
        }
        std::cout << "Load pass" << std::endl;
    }

    NS_NURBS::Segment2NodeMapT nurbsDatas;
    //Get
    {
        NS_NURBS::NurbsLoader::Instance().Get(nurbsDatas);
        std::cout << "Get pass" << std::endl;
        std::cout << "Get data.size()|"<<nurbsDatas.size()<<std::endl;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud_total;
    std::stringstream ss; 
    int seq_count=0;
    for (auto it = nurbsDatas.begin(); it !=nurbsDatas.end(); it++)
    {
        seq_count++;
        int line_count=0;
        ss<<"newseq"<<std::endl;
        ss<<it->first<<std::endl;
        ss<<it->second->sReferencePoint<<std::endl;
        for (auto it1 = it->second->equationDesc.begin() ; it1 !=it->second->equationDesc.end(); it1++){
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            bool isDash;
            Conv_FormatNURBS(it1->second, cloud, isDash);
            if (cloud.size()>0){
                cloud_total.insert(cloud_total.end(), cloud.begin(), cloud.end());
                ss<<"newline"<<std::endl;
                if (isDash){
                    ss<<"dash"<<std::endl;
                }else{
                    ss<<"solid"<<std::endl;
                }
                for (int i=0;i<cloud.size();i++){
                    ss<<cloud[i].x<<" "<<cloud[i].y<<" "<<cloud[i].z<<std::endl;
                }
                line_count++;
            }
        }
        std::cout<<"seq: "<<it->first<<" | "<<"ref: "<<it->second->sReferencePoint<<" | "<<"line count: "<<line_count<<std::endl;
    }
    std::ofstream out("painting.txt");
    out<<ss.str();
    out.close();
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZRGB>("chamo.ply",cloud_total, true, false);
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "./example dbpath [-v]"<<std::endl;
        return -1;
    }
    std::string sDBPath = argv[1];

    
    if (argc > 2)
    {
        std::string sV = argv[2];
        gDebug = (sV == "-v");
    }

    NS_NURBS::ConfT conf;
    conf.sDbPath = sDBPath;
    if (gDebug)
    {
        conf.lvl = NS_NURBS::LOGLVL_DEBUG;
    }

    Test_LoadNurbs(conf);
    
    return 0;
}
