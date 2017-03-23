/**
 *******************************************************************************
 *                       Continental Confidential
 *                  Copyright (c) Continental, LLC. 1995-2016
 *
 *      This software is furnished under license and may be used or
 *      copied only in accordance with the terms of such license.
 *******************************************************************************
 * @file   AlgoInterfaceImp.h
 * @brief  file of implementation of vehicle SLAM control system interface. These
 *         interfaces are inherited from global interfaces and just used only for/Volumes/chamo/dataset/overpass_west_to_south 2/overpass_west_to_south.mp4/Volumes/chamo/dataset/overpass_west_to_south 2/overpass_west_to_south.mp4
 *         debugging of vehicle SLAM.
 *
 * Change Log:
 *      Date              Who            What
 *      2016.10.19       <Li.Wang>      Created
 *
 *******************************************************************************
 */

#ifndef _I_ALGO_IMP_H_
#define _I_ALGO_IMP_H_



#include <string>
#include <memory>

#include <opencv/cv.h>

#include "algoInterface/IGps.h"

#include "typeDef.h"

#include "algoInterface/KeyFrameDef.h"


namespace algo {


class AlgoGPS : public IGps
{
	public:
        static algo::IGps* createInstance(const std::string &strGPS = "", algo::IGps * Gps = NULL);

		virtual bool getRefPos(OUT cv::Point3d &gps);
		virtual bool getRelPos(IN int32_t frmIdx, OUT cv::Point3f &gps);
		virtual bool getTimeStamp(IN int32_t frmIdx, OUT uint32_t &timeStamp);
	
		//bool KalmanFilter(std::vector<cv::Point3f> &vRelGPSP3f_);
		//void GpsSmooth(std::vector<cv::Point3f> &vRelGPSP3f_);
		bool generateGpsKml(const std::string &strFileName,
							const std::vector<algo::GPSData> &vecGpsData);
		
		std::vector<cv::Point3f> & getRelGpsVec();
		cv::Point3d & getRefGps();
	
	private:
		AlgoGPS(const std::string &strGPS);
        static algo::IGps*    m_pInstance;
		bool					 bInitialized_;
		cv::Point3d 			 refGPSP3d_;
		std::vector<cv::Point3f> vRelGPSP3f_;
		std::vector<uint32_t>	 vTimeStamp_;
}; // class AlgoGPS

} // namespace algo

#endif // _I_ALGO_IMP_H_
