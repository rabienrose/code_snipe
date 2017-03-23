#include <iostream>
#include "gazebo/common/Plugin.hh"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/ImuSensor.hh>
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
    
    class myIMU : public SensorPlugin
    {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
        printf("IMU Load!\n");
        imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
        updateConnection = _sensor->ConnectUpdated(std::bind(&myIMU::OnUpdate, this));
        myfile_g.open ("/media/psf/chamo/working/gazebo/recording/gyro.txt");
        myfile_a.open ("/media/psf/chamo/working/gazebo/recording/acc.txt");
    }
    public: void OnUpdate(){
        common::Time cureTime = imuSensor->LastMeasurementTime();
        double timef = cureTime.Double();
        std::stringstream ss;
        ignition::math::Vector3d gyro = imuSensor->AngularVelocity(true);
        ignition::math::Vector3d acce = imuSensor->LinearAcceleration(true);
        myfile_a << acce.X()<<","<< acce.Y()<<","<< acce.Z()<<","<<timef<<std::endl;
        myfile_g << gyro.X()<<","<< gyro.Y()<<","<< gyro.Z()<<","<<timef<<std::endl;
    }
    private:
        sensors::ImuSensorPtr imuSensor;
        event::ConnectionPtr updateConnection;
        std::ofstream myfile_a;
        std::ofstream myfile_g;
    };

    //Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(myIMU)
    
}
