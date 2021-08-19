// implemented by Jungill Kang, 8.AUG.2021

#ifndef GAZEBO_PLUGINS_WCFANFORCEPLUGIN_HH_
#define GAZEBO_PLUGINS_WCFANFORCEPLUGIN_HH_

#include <ignition/transport/Node.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE FanForcePlugin : public ModelPlugin
  {
    public: FanForcePlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnVelMsg(ConstPosePtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr forceSub;

    private: physics::ModelPtr model;
    private: physics::JointPtr leftJoint, rightJoint;
    private: event::ConnectionPtr updateConnection;
    private: double wheelSpeed[4];
    private: double wheelSeparation;
    private: double robotSeperation;
    private: double wheelRadius;
    private: common::Time prevUpdateTime;

    private: physics::LinkPtr link, leftWheelLink, rightWheelLink;

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    // brief Ignition transport node
    private: ignition::transport::Node nodeIgn;
  };
}

#endif