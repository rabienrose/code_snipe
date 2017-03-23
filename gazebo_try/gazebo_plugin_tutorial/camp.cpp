#include <iostream>
#include "gazebo/common/Plugin.hh"
#include "gazebo/plugins/CameraPlugin.hh"
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>


int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    return 0;
}

namespace gazebo
{
    
    class myCamera : public SensorPlugin
    {
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
        printf("Camera Load!\n");
        camSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
        updateConnection = _sensor->ConnectUpdated(std::bind(&myCamera::OnUpdate, this));
        myfile.open ("/media/psf/chamo/working/gazebo/recording/cam_time.txt");
        
    }
    public: void OnUpdate(){
        common::Time cureTime = camSensor->LastMeasurementTime();
        double timef = cureTime.Double();
        std::stringstream ss;
        static int img_count=0;
        ss<<"/media/psf/chamo/working/gazebo/recording/image/img_"<<std::setw (6)<< std::setfill ('0')<<img_count<<".jpg";
        //std::cout<<ss.str()<<std::endl;
        camSensor->SaveFrame(ss.str().c_str());
        myfile<<timef<<","<<img_count<<std::endl;
        img_count++;
    }
    private:
        sensors::CameraSensorPtr camSensor;
        event::ConnectionPtr updateConnection;
        std::ofstream myfile;
    };

    //Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(myCamera)
    
}
