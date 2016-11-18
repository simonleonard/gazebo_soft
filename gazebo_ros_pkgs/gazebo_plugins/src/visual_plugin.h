//A visual plugin for allowing the interaction of a mesh with a softbody. 
//Hudson Thomas
//June 22, 2016

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <BulletLink.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <btSoftBody.h>
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include <gazebo/math/Vector2.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE Visual_Plugin : public SystemPlugin
  {

  public: void Load(rendering::VisualPtr _visual,sdf::ElementPtr _sdf);

  public: void OnUpdate(const common::UpdateInfo & /*_info*/);

  public: void OnTestMsg( const boost::shared_ptr<msgs::Vector2d const> &_msg);

  //subscriber node declaration
  private: transport::NodePtr node;
  private: transport::SubscriberPtr testSub;
 
  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };
}
