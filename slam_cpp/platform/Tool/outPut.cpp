#include "outPut.h"
#include "comAglo.h"

#include <iomanip>

namespace ygomi
{
    std::unique_ptr<outPut> outPut::m_pInstance = nullptr;
	outPut::outPut(std::string &debugFolder)
	{
		bf::path targetPath = bf::path(debugFolder);
		targetPath.remove_filename();

		if (!assurePathExists(targetPath.c_str()) )
		{
		    throw std::string("Could not open file ") + debugFolder;
		}
        std::cout << "outPut is created"<<std::endl;
	}
	
	outPut::~outPut()
	{
        std::cout << "outPut is destructed"<<std::endl;
	}

	bool outPut::assurePathExists(const std::string &strPath)
	{
		bool bRet = false;

		bf::path targetPath = bf::path(strPath);
		if ( !targetPath.empty() )
		{
			if ( bf::exists(targetPath) )
			{
				bRet = true;
			}
			else
			{
				bRet = bf::create_directories(targetPath);
			}
		}
		else
		{
			// empty string represents current directory
			bRet = true;
		}

		return bRet;
	}

	ygomi::outPut* outPut::createInstance(std::string &debugFolder)
	{
        if(!m_pInstance)
            m_pInstance = std::unique_ptr<outPut>(new outPut(debugFolder));
        return m_pInstance.get();
	}
	
	bool outPut::generateSlamKml(const std::string &strFileName,
									const std::vector<cv::Mat> &vecTrajectory)
	{
		bool bRet = false;
	
		std::string kmlPath = strFileName;
		bf::path targetPath = bf::path(kmlPath);
		targetPath.remove_filename();
	
		if (assurePathExists(targetPath.c_str()) )
		{
			std::ofstream outFile(kmlPath.c_str());
	
			if ( outFile.is_open() )
			{
				// opening marks info of KML file
				const std::string STR_OPEN_MARK = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
						"<kml>\n"
						"<Document>\n"
						"\t<Style id=\"redLineStyle\"><LineStyle><color>ff0000ff</color></LineStyle></Style>\n"
						"\t<StyleMap id=\"redLine\"><Pair><key>normal</key><styleUrl>#redLineStyle</styleUrl></Pair></StyleMap>\n"
						"\t<Placemark>\n"
						"\t\t<name>SLAM</name>\n"
						"\t\t<styleUrl>#redLine</styleUrl>\n"
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
	
				// alignment
				outFile << "\t\t\t";
	
				// slam trajectory
				const int PRECISION_BIT = 8;
				std::vector<cv::Mat>::const_iterator citT = vecTrajectory.begin();
				for ( ; citT != vecTrajectory.end(); ++citT )
				{
					assert(1 == citT->cols || 1 == citT->rows);
	
					cv::Mat matTraj = *citT;
					if ( 1 != matTraj.cols )
					{
						// transpose
						matTraj.t();
					}
	
					// x <--> lon
					// z <--> lat
					outFile << std::fixed
							<< std::setprecision(PRECISION_BIT) << matTraj.at<double>(0, 0) << ","
							<< std::setprecision(PRECISION_BIT) << matTraj.at<double>(2, 0) << ","
							<< std::setprecision(PRECISION_BIT) << 0<< " ";
				}
	
				outFile << std::endl << STR_CLOSE_MARK;
	
				bRet = true;
			}
		}
		else
		{
			assert(false);
		}
	
		return bRet;
	}
	
}
