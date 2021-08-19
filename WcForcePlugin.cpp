// implemented by Jungill Kang, 8.AUG.2021

// to suscefully set the specific ink
// <link_name> link name </link_name>

// to set the max force value of the propeller
// <max_force> force </max_force>


#include "gazebo/physics/physics.hh"
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>

#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ignition/math/Pose3.hh>

#include "WcForcePlugin.hpp"

#include <memory>
#include <string>

enum AXIS
{
  X, Y, Z
};

namespace gazebo_plugins
{
class WcForcePluginPrivate
{
  // Pointer to link, where the force is applied
  public: gazebo::physics::LinkPtr link_;

  // gazebo and ros interfacing node
  public: gazebo::transport::NodePtr node_;

  // Wrench subscriber
  public: rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;

  // Wrench msg container
  public: geometry_msgs::msg::Wrench wrench_msg_;

  // Pointer to the update event connection
  public: gazebo::event::ConnectionPtr update_connection_;

  public: gazebo::common::Time prev_update_time_;

  // Indicates that the force should be applied on the world frame instead of the link frame
  bool force_on_world_frame_;
};

/////////////////////////////////////////////////
WcForcePlugin::WcForcePlugin()
: impl_(std::make_unique<WcForcePluginPrivate>())
{
}

/////////////////////////////////////////////////
WcForcePlugin::~WcForcePlugin()
{
}

/////////////////////////////////////////////////
void WcForcePlugin::Load(gazebo::physics::ModelPtr model,
                           sdf::ElementPtr sdf)
{
  // make ROS logger
  auto logger = rclcpp::get_logger("Wc Force Plugin");

  // Node for interfacing between ros and gazebo

  // find the target link
  if (!sdf->HasElement("link_name")){
    RCLCPP_ERROR(logger, "WcForce plugin missing <link> element\n");
    return;
  }

  auto link_name = sdf->GetElement("link")->Get<std::string>();

  impl_->link_ = model->GetLink(link_name);

  if (!impl_->link_){
    RCLCPP_ERROR(logger, "link named: %s does not exist\n", link_name.c_str());
    return;
  }

  // find the reference frame

  
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&WcForcePlugin::OnUpdate, impl_));
}

/////////////////////////////////////////////////

GZ_REGISTER_MODEL_PLUGIN(WcForcePlugin)
} // namespace gazebo