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
#include "gazebo/util/system.hh"
#include "gazebo/rendering/rendering.hh"
//#include "visual_plugin.h"
//#include <gazebo/math/Vector2.hh>



#include "gazebo/gui/GuiIface.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/util/system.hh"
namespace gazebo
{
  class Visual_Plugin : public VisualPlugin
  {

  public: Visual_Plugin(){
    std::cout << "constructor" << std::endl;
  }

  public: ~Visual_Plugin(){
    std::cout << "destructor" << std::endl;
  }

  public: void Load( 	rendering::VisualPtr  	_visual,
			sdf::ElementPtr  	_sdf 
			){

    std::cout<<"loading visual plugin"<<std::endl;
}


  public: void Init()
  {   
    //initialize communication
    this->node = transport::NodePtr(new transport::Node());  
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&Visual_Plugin::InitAtPreRenderEvent, this ) );//, _1));
    //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Visual_Plugin::InitAtRenderEvent, this, _1)); 
    std::cout<<"connection updated"<<std::endl;

    this->node->Init(); //initializes the node to the first namespace in Master by defaul, can also specify "world name
    std::cout<<"Node initialized"<<std::endl;
    std::cout<<"node subscribed"<<std::endl;
    this->testSub = this->node->Subscribe("~/sb_pose", &Visual_Plugin::OnTestMsg, this);
  }
      

  public: void InitAtPreRenderEvent()//const common::UpdateInfo & /*_info*/)
  {
    //this is where we actually render our new position stuff. 
  }

  public: void OnTestMsg(ConstPointCloudPtr &_msg)
  {
    //std::cout<< _msg->points_size();
    //gazebo::common::Time::MSleep(10000);
  }


    //2 helper methods to copy the msg: the first puts it into a tVector3Array (http://bulletphysics.org/Bullet/BulletFull/classbtSoftBody.html#a8a258d7e5f9bb847ed754e5aa49e6a0e)
    //    and the other puts it into an array of btVector3's (http://bulletphysics.org/Bullet/BulletFull/classbtVector3.html)
    public: std::vector<msgs::Vector3d> readMsg1(ConstPointCloudPtr &_msg)
  {
    int i;
    std::vector<msgs::Vector3d> newArr;
    for(i = 0; i<_msg->points_size();i++)
      {
        newArr.push_back(_msg->points(i));
      } 
    return newArr;
    }
/*
  public: btVector3 * readMsg2(const boost::shared_ptr<gazebo::msgs::PointCloud const> &_msg)
    {
    int i;

    int size = _msg->points_size();
    static btVector3 newArr[size];
    for(i = 0; i<size;i++)
      {
        newArr[i]= (_msg->points(i));
      } 
    return newArr;
    }*/


    
  //subscriber node declaration
private: transport::NodePtr node;
private: transport::SubscriberPtr testSub;
 
  // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;

};
// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(Visual_Plugin)
}
