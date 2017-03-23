#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <Eigen/Eigen>

namespace gazebo
{
    math::Quaternion lookAt(math::Vector3 lookPos, math::Vector3 camPos){
        Eigen::Vector3f lookPos_e(lookPos.x, lookPos.y, lookPos.z);
        Eigen::Vector3f camPos_e(camPos.x, camPos.y, camPos.z);
        Eigen::Vector3f f_dir=lookPos_e - camPos_e;
        f_dir = f_dir.normalized();
        Eigen::Vector3f up(0,0,1);
        Eigen::Vector3f r_dir = up.cross(f_dir);
        r_dir = r_dir.normalized();
        Eigen::Vector3f u_dir =f_dir.cross(r_dir);
        u_dir = u_dir.normalized();
        Eigen::Matrix3f mat;
        mat.col(0) =f_dir;
        mat.col(1) =r_dir;
        mat.col(2) =u_dir;
        
        Eigen::Quaternionf q(mat);
        //std::cout<<mat<<std::endl;
        math::Quaternion quat(q.w(),q.x(),q.y(),q.z());
        //std::cout<<quat<<std::endl;
        return quat;
        
    }
    class ModelPush : public ModelPlugin{
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
            printf("Model Load!\n");
            this->model = _parent;
            math::Pose pose;
            pose.pos= math::Vector3(0,30, 2);
            math::Quaternion rot = lookAt(math::Vector3(0,0,2), pose.pos);
            //pose.rot =math::Quaternion(1,0,0,0);
            //this->model->SetAngularVel(math::Vector3(0, 0, 1));
            this->model->SetLinearVel(math::Vector3(0, -1, 0));
            pose.rot = rot;
            this->model->SetWorldPose(pose);
            this->model->SetSelfCollide(false);
            this->model->SetGravityMode(false);
            updateCount= 0;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
            myfile.open ("/media/psf/chamo/working/gazebo/recording/body.txt");
        }
        
        void OnUpdate(const common::UpdateInfo &_info){
            common::Time cureTime = _info.simTime;
            double timef = cureTime.Double();
            //float x=rot_r*std::sin(timef*rot_v);
            //float y=rot_r*std::cos(timef*rot_v);
            //float x= x+timef*1;
            //float y= y-//timef*2;
            //this->model->SetLinearVel(math::Vector3(0, y, 0));
            //this->model->SetAngularVel(math::Vector3(0, 0, x/2));
            //if(updateCount ==100){
                //this->model->SetLinearVel(math::Vector3(1, 0, 0));
                //this->model->SetAngularVel(math::Vector3(1, 0, 0));
            //}

            
            if(updateCount%100==0){
                const math::Pose pose_out = this->model->GetWorldPose();
                math::Vector3 v_out = this->model->GetWorldLinearVel();
                myfile<<timef<<","<<pose_out.pos.x<<","<<pose_out.pos.y<<","<<pose_out.pos.z<<","<<
                                    pose_out.rot.w<<","<<pose_out.rot.x<<","<<pose_out.rot.y<<","<<pose_out.rot.z<<","<<
                                    v_out.x<<","<<v_out.y<<","<<v_out.z<<","<<updateCount<<std::endl;
            }
            updateCount++;
            
            //std::cout<<"iter: "<<updateCount<<std::endl;
        }
        
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        std::ofstream myfile;
        long updateCount;
        float rot_v=1;
        float rot_r=5;
    };
    
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
