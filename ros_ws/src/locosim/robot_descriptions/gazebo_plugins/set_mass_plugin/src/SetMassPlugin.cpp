#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sstream>
#include <gazebo_msgs/SetLinkProperties.h>

namespace gazebo
{
  class DynamicMassPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      this->model = _model;

      // Set up ROS node
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
      }

      this->rosNode = new ros::NodeHandle();

      // Advertise the set_mass service
      this->setMassService = this->rosNode->advertiseService("/set_mass", &DynamicMassPlugin::SetMassCallback, this);

      std::cout << "\033[31mDynamicMassPlugin loaded and ready to set mass. \033[0m" << std::endl;
    }

    bool SetMassCallback(gazebo_msgs::SetLinkProperties::Request &req, gazebo_msgs::SetLinkProperties::Response &res)
    {
      this->link = this->model->GetLink(req.link_name);
      if (this->link)
      {
        // Set the new mass (you can dynamically calculate or pass this value)

        physics::InertialPtr inertial = this->link->GetInertial();
        inertial->SetMass(req.mass);

        // Apply the new inertial properties
        this->link->SetInertial(inertial);
        this->link->UpdateMass();
        std::cout << "\033[31mLink mass set to: " << req.mass << "\033[0m" << std::endl;
      }
      else
      {
        ROS_ERROR("DynamicMassPlugin: Link not found when setting mass.");
        return false;
      }

      return true;
    }

    ~DynamicMassPlugin()
    {
      // Clean up
      delete this->rosNode;
    }

  private:
    physics::ModelPtr model;
    physics::LinkPtr link;
    ros::NodeHandle *rosNode;
    ros::ServiceServer setMassService;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(DynamicMassPlugin)
}
