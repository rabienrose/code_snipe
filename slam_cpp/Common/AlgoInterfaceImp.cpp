/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   AlgoInterfaceImp.cpp
 * @brief  file of implementation of vehicle SLAM control system interface. These
 *         interfaces are inherited from global interfaces and just used only for
 *         debugging of vehicle SLAM.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.10.19       <Li.Wang>      Created
 *
 *******************************************************************************
 */

#include "AlgoInterfaceImp.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <string>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

//#include "algo_bf.hpp"
//#include "algo_log.hpp"
//#include "serializeImp.hpp"
//#include "cvFileStorageImp.hpp"
#include <cmath>

#include <iomanip>
#include "comAglo.h"

namespace algo {
	algo::IGps* AlgoGPS::m_pInstance = nullptr;
	
	// class AlgoGPS
	AlgoGPS::AlgoGPS(const std::string &strGPS)
	{
		bInitialized_ = false;
	
		int idx = 0;
		std::fstream fs;
		fs.open(strGPS, std::fstream::in);
		if (fs.is_open())
		{
			vRelGPSP3f_.clear();
			vTimeStamp_.clear();
	
			while (!fs.eof())
			{
				char lineData[256];
				fs.getline(lineData, 256);
	
				if (lineData[0] != '\0')
				{
					uint32_t timeStamp = 0;
					if(idx == 0)
					{
						cv::Point3d refGPS;
						sscanf(lineData, "$GPGGA,%d,%lf,%lf,%lf", &timeStamp, &refGPS.y, &refGPS.x, &refGPS.z);
						refGPSP3d_ = refGPS;
					}
					else
					{
						cv::Point3f relGPS;
	
						sscanf(lineData, "$GPGGA,%d,%f,%f,%f", &timeStamp, &relGPS.y, &relGPS.x, &relGPS.z);
	
						vRelGPSP3f_.push_back(relGPS);
						vTimeStamp_.push_back(timeStamp);
					}
	
					idx++;
				}
			}
	
			if (!vRelGPSP3f_.empty())
			{
				bInitialized_ = true;
			}
	
			fs.close();
		}
		else
		{
			std::cout<< "Failed to open GPS file " << strGPS << std::endl;
		}
	
		if (bInitialized_)
		{
		}
	 
	}
	
	algo::IGps* AlgoGPS::createInstance(const std::string &strGPS, algo::IGps * Gps)
	{
	    
        if(!m_pInstance)
		{
			if (Gps)
			{
				m_pInstance	= Gps; 
			}
			else
			{
				m_pInstance = dynamic_cast<algo::IGps*>(new AlgoGPS(strGPS));
			}
		}
        return m_pInstance;
	}
	
	bool AlgoGPS::getRefPos(OUT cv::Point3d &gps)
	{
		if (bInitialized_)
		{
			gps = refGPSP3d_;
		}
	
		return bInitialized_;
	}
	
	bool AlgoGPS::getRelPos(IN roadDBCore::int32_t frmIdx, OUT cv::Point3f &gps)
	{
		bool ret = true;
		if (vRelGPSP3f_.empty() || (frmIdx < 0) ||
			(frmIdx > (int32_t)(vRelGPSP3f_.size() - 1)))
		{
			ret = false;
		}
		else
		{
			gps = vRelGPSP3f_[frmIdx];
		}
	
		return ret;
	}
	
	bool AlgoGPS::getTimeStamp(IN roadDBCore::int32_t frmIdx,
								OUT roadDBCore::uint32_t &timeStamp)
	{
		bool ret = true;
		if (vTimeStamp_.empty() || (frmIdx < 0) ||
			(frmIdx > (int32_t)(vTimeStamp_.size() - 1)))
		{
			ret = false;
		}
		else
		{
			timeStamp = vTimeStamp_[frmIdx];
		}
	
		return ret;
	}
	
	bool AlgoGPS::generateGpsKml(const std::string &strFileName,
								   const std::vector<algo::GPSData> &vecGpsData)
	{
		bool bRet = false;
	
		std::string kmlPath = "./" + strFileName;
		bf::path targetPath = bf::path(kmlPath);
		targetPath.remove_filename();
	
		if ( CommonAlgo::assurePathExists(targetPath.c_str()) )
		{
			std::ofstream outFile(kmlPath.c_str());
	
			if ( outFile.is_open() )
			{
				// opening marks of KML file
				const std::string STR_OPEN_MARK = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
						"<kml>\n"
						"<Document>\n"
						"\t<Style id=\"greenLineStyle\"><LineStyle><color>ff00ff00</color></LineStyle></Style>\n"
						"\t<StyleMap id=\"greenLine\"><Pair><key>normal</key><styleUrl>#greenLineStyle</styleUrl></Pair></StyleMap>\n"
						"\t<Placemark>\n"
						"\t\t<name>GPS</name>\n"
						"\t\t<styleUrl>#greenLine</styleUrl>\n"
						"\t\t<LineString>\n"
						"\t\t\t<altitudeMode>relativeToGround</altitudeMode>\n"
						"\t\t\t<coordinates>\n";
	
				// close marks of KML file
				const std::string STR_CLOSE_MARK = "\t\t\t</coordinates>\n"
						"\t\t</LineString>\n"
						"\t</Placemark>\n"
						"</Document>\n"
						"</kml>\n";
	
				outFile << STR_OPEN_MARK;
	
				// coordinates alignment
				outFile << "\t\t\t";
	
				// GPS trajectory
				const int PRECISION_BIT = 8;
				std::vector<algo::GPSData>::const_iterator citGPS = vecGpsData.begin();
				for ( ; citGPS != vecGpsData.end(); ++citGPS )
				{
					const algo::GPSData &gpsData = *citGPS;
	
					outFile << std::fixed
							<< std::setprecision(PRECISION_BIT) << gpsData.lon << ","
							<< std::setprecision(PRECISION_BIT) << gpsData.lat << ","
							<< std::setprecision(PRECISION_BIT) << 0 << " ";
				}
	
				outFile << std::endl << STR_CLOSE_MARK;
	
				bRet = true;
			}
		}
	
		return bRet;
	}
	
	std::vector<cv::Point3f> & AlgoGPS::getRelGpsVec()
	{
	    return vRelGPSP3f_;
	}
	
	cv::Point3d & AlgoGPS::getRefGps()
	{
		return refGPSP3d_;
	}
}

