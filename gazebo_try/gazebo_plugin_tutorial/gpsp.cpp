#include <iostream>
#include "gazebo/common/Plugin.hh"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <stdio.h>
#include <gazebo/gazebo.hh>
#include <fstream>


int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    return 0;
}

namespace gazebo
{
    
    class myGPS : public SensorPlugin
    {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
        printf("GPS Load!\n");
        gpsSensor = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
        updateConnection = _sensor->ConnectUpdated(std::bind(&myGPS::OnUpdate, this));
        //myfile.open ("/media/psf/chamo/working/gazebo/recording/gps.txt");
    }
    public: void OnUpdate(){
        common::Time cureTime = gpsSensor->LastMeasurementTime();
        double timef = cureTime.Double();
        std::stringstream ss;
        ignition::math::Angle lon = gpsSensor->Longitude();
        ignition::math::Angle lat = gpsSensor->Latitude();
        double alt = gpsSensor->Altitude();
        //myfile << lon.Radian()<<","<< lat.Radian()<<","<< alt<<","<<timef<<std::endl;;
    }
    private:
        sensors::GpsSensorPtr gpsSensor;
        event::ConnectionPtr updateConnection;
        std::ofstream myfile;
    };

    //Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(myGPS)
    
}
