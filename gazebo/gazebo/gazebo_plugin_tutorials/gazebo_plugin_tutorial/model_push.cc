
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <BulletLink.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <btSoftBody.h>
#include <btSoftBodyHelpers.h>
#include "LinearMath/btScalar.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include <gazebo/math/Vector2.hh>
 
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      std::cerr<<"asdfasfdsafLoad ModelPush"<<std::endl;
      // Store the pointer to the model
      this->model = _parent;
      this->link = this->model->GetLink("link");

      
      //create softbody
      btSoftBodyWorldInfo worldInfo = btSoftBodyWorldInfo();
      btSoftBodyWorldInfo* worldInfoP = &worldInfo;
      const btScalar	s=6;
      const btScalar	h=2;
      const int		r=16;	
      const btVector3	p[]={	btVector3(+s,h,-s),
				btVector3(-s,h,-s),
				btVector3(+s,h,+s),
				btVector3(-s,h,+s)};
      this->b_link = new physics::BulletLink (this->model);
      /*
      this->b_link->softLink = btSoftBodyHelpers::CreatePatch( worldInfo,
							       p[0],
							       p[1],
							       p[2],
							       p[3],
							       r,
							       r,
							       1+2+4+8,
							       true );
      */
      

      btScalar* vertices;
asdfasfd      int* triangles;
      int ntriangles;
       this->b_link->softLink = 
	btSoftBodyHelpers::CreateFromTriMesh( worldInfo,
					      
      std::cout<<"SOFT HELPER JUST MADE THE PATCH^^^^^"<<std::endl;

      //testing
      this->b_link->softLink->m_tag = (void*) 'f';
      std::cout<<btSoftBodyHelpers::CalculateUV(10, 9, 8, 7, 0)<<std::endl;
      btSoftBody::Material* mat;
      mat->m_kLST = 0.5;
      mat->m_kAST = 0.5;
      mat->m_kVST = 0.5;
      this->b_link->softLink->appendLink(-1, mat);
      std::cout<<"A^^^^^"<<std::endl;

      this->b_link->softLink->m_links.at(0).m_n[0]->m_x.setX(3.0);
      std::cout<<"B^^^^^"<<std::endl;

      std::cout<<this->b_link->softLink->m_links.at(0).m_n[0]->m_x.getX()<<std::endl;

      std::cout<<"C^^^^^"<<std::endl;

      //initialize communication
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(); //initializes the node to the first namespace in Master by default, can also specify "world name"
      this->sbPub = this->node->Advertise<msgs::PointCloud>("~/sb_pose", 10);
     
      addPoseToMsg(81);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1));
      // std::cout << "ModelPush::Load f" << std::endl;
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //addPoseToMsg(81);
      // this->sbPub->Publish(sbpose_msg);
      
      //this->b_link->softLink->setTotalMass(5.0, false);
      // printf("mass = %f",this->b_link->softLink->getTotalMass());

      
    }
    

    public: void addPoseToMsg(int numNodes)
    {
      //write the soft body position data to the message to be published
      int i;	 
      //msgs::Vector3d* nodePoints = sbpose_msg.add_points();
      //*nodePoints =  msgs::Vector3d();
      std::cout<<"about to set message"<<std::endl;
      std::cout<< *(char*) (&this->b_link->softLink->m_tag) <<std::endl;
      

      //std::cout<<this->b_link->softLink->m_pose.m_pos.at(1).getX() <<std::endl;
      std::cout<<this->b_link->softLink->m_links.at(0).m_n[0]->m_x.getX() <<std::endl;

      std::cout<<"about to set message"<<std::endl;

      /*const ignition::math::Vector3<double>& nodePoints = ignition::math::Vector3<double>(
						      this->b_link->softLink->m_pose.m_pos.at(1).getX(),
						      this->b_link->softLink->m_pose.m_pos.at(1).getY(),
						      this->b_link->softLink->m_pose.m_pos.at(1).getZ());
      std::cout<<"about to set message 3"<<std::endl;
    //msgs::Set(sbpose_msg.add_points(), nodePoints);
      /*for(i = 0; i< numNodes; i++)
	{
	  msgs::Vector3d* nodePoints = sbpose_msg.add_points();
	  nodePoints->set_x(this->b_link->softLink->m_pose.m_pos.at(i).getX());
	  nodePoints->set_y(this->b_link->softLink->m_pose.m_pos.at(i).getY());
	  nodePoints->set_z(this->b_link->softLink->m_pose.m_pos.at(i).getZ());
	} 
      */
    }
    
    //Pointer to a BulletLink
    gazebo::physics::BulletLink* b_link;
    
    //Pointer to the link
    private: physics::LinkPtr link; 

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;


    // Node for publishing the mesh position data
    private: transport::NodePtr node;

    //publisher for fluid visual messages
    private: transport::PublisherPtr sbPub;

    //The message containing the 3d coordinate of each node in the softbody
    msgs::PointCloud sbpose_msg;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
