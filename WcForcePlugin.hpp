// implemented by Jungill Kang, 8.AUG.2021
// ref : https://github.com/ros-simulation/gazebo_ros_pkgs/blob/foxy/gazebo_plugins/include/gazebo_plugins/gazebo_ros_force.hpp

#ifndef GAZEBO_PLUGINS_WCFORCEPLUGIN_HH_
#define GAZEBO_PLUGINS_WCFORCEPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include "gazebo/gazebo.hh"

#include <geometry_msgs/msg/wrench.hpp>

namespace gazebo_plugins
{
class WcForcePluginPrivate;

class WcForcePlugin : public gazebo::ModelPlugin
{
  public: WcForcePlugin();
  public: virtual ~WcForcePlugin();
  protected: virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  protected: virtual void Init();
  protected: virtual void OnUpdate();

  private: void OnWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg);

  // pimpl pattern
  private: std::unique_ptr<WcForcePluginPrivate> impl_;
};
}

#endif