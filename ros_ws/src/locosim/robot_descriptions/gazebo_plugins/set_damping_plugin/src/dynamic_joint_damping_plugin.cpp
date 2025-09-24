#include <ros/ros.h>
#include <set_damping_plugin/SetFloat.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class DynamicJointDampingPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
      }

      this->rosNode = new ros::NodeHandle();
      // Advertise the set_joint_damping service
      this->setDampingService = this->rosNode->advertiseService("/set_joint_damping", &DynamicJointDampingPlugin::SetDampingCallback, this);
      std::cout << "\033[31mDynamicDampingPlugin loaded and ready to set damping. \033[0m" << std::endl;

      // Get all joints in the model
      joints = this->model->GetJoints();
      std::cout << "\033[31mFound " << joints.size() << " joints in the model \033[0m" << std::endl;
    }

    bool SetDampingCallback(set_damping_plugin::SetFloatRequest &req, set_damping_plugin::SetFloatResponse &res)
    {

      double new_damping = req.value;
      // Iterate through each joint
      for (auto &joint : joints)
      {
        joint->SetDamping(0, new_damping); // first entry is Dof of the joint use 0 for prismatic/revolute joints 1 for ball joints
        joint->Update();                   // Force physics update

        //  Explicitly set damping using ODE parameters (for ODE physics)
        // this->joint->SetParam("damping", new_damping);
      }
      std::cout << "\033[31mDynamicDampingPlugin:Damping set to: " << new_damping << "\033[0m" << std::endl;
      return true;
    }

    ~DynamicJointDampingPlugin()
    {
      delete this->rosNode;
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    ros::NodeHandle *rosNode;
    ros::ServiceServer setDampingService;
    std::vector<physics::JointPtr> joints;
  };

  GZ_REGISTER_MODEL_PLUGIN(DynamicJointDampingPlugin)
}
