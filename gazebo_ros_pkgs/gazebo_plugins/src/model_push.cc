/*
Description: his plugin creates a bullet softbody and puts it into the gazebo world. It also creates and 
sends a gazebo message containing the 3d position of each node in the softbody so that it can
be rendered by a visual plugin. 

Author: Hudson Thomas
Date: July 25, 2016
*/


#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/bullet/BulletLink.hh>
#include <gazebo/common/common.hh>
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <typeinfo>
 
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
  public:
    ModelPush() {}

 ////////////////////////////////////////////////////////////////////////////////
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      std::cout << "LOAD" << std::endl;
      // initialize members
      this->model = _parent;
      this->b_link.reset(new physics::BulletLink (this->model));
      this->b_link->Load(_sdf->GetParent());
      this->b_link->Init();
      
      //create softbody
      btSoftBodyWorldInfo* worldInfo = new btSoftBodyWorldInfo();
      worldInfo->m_sparsesdf.Initialize();
      worldInfo->m_sparsesdf.Reset();
      this->softBody.reset( Init_ClothAttach(worldInfo) );

      //initialize communication
      this->node = transport::NodePtr(new transport::Node()); 
      this->node->Init();
      this->sbPub = this->node->Advertise<msgs::PointCloud>("~/sb_pose", 100);

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelPush::OnUpdate, this, _1)); 
    }

///////////////////////////////////////////////////////////////////////////////////////
  public: void OnUpdate(const common::UpdateInfo &)
    {    

      //this->softBody->cutOnContact(&(*(this->softBody)));
      //std::cout<< typeid(*(this->softBody)).name()<<std::endl;
      //Message containing the 3d coordinate of each node in the softbody
      msgs::PointCloud nodePose_msg;
      //Message containing the indices of the nodes between which the links are cut
      //msgs::Vector2d linksCut_msg;

      // numbers of faces x 3
      /*
      const size_t ibufCount = softBody->m_faces.size()*3;
      unsigned short faces[ibufCount];
      {
	int j = 0;
	for( int i=0; i< softBody->m_faces.size(); i++ ){
	  faces[j++] = (softBody->m_faces[i].m_n[0] -  &(softBody->m_nodes[0]));
	  faces[j++] = (softBody->m_faces[i].m_n[1] -  &(softBody->m_nodes[0]));
	  faces[j++] = (softBody->m_faces[i].m_n[2] -  &(softBody->m_nodes[0]));
	}
      }
      */

      //copy the 3d position of each node of the softbody into the message.  
      for(int i = 0; i<softBody->m_nodes.size(); i++)
	{
	  double deltaX = this->softBody->m_nodes.at(i).m_x.getX();
	  double deltaY = this->softBody->m_nodes.at(i).m_x.getY();
	  double deltaZ = this->softBody->m_nodes.at(i).m_x.getZ();
	    
	  const ignition::math::Vector3<double>& nodePoints = ignition::math::Vector3<double>(deltaX, deltaY, deltaZ);
	  msgs::Set(nodePose_msg.add_points(), nodePoints);
	}    

      /*   
      //copy the indices of the links that need to be cut into the linksCut_msg message
      for(i = 0; i<this->softBody->cutLinks.size; i++)
	{
	  double deltaX = this->softBody->m_nodes.at(i).m_x.getX();
	  double deltaY = this->softBody->m_nodes.at(i).m_x.getY();
	  double deltaZ = this->softBody->m_nodes.at(i).m_x.getZ();
	    
	  const ignition::math::Vector3<double>& nodePoints = ignition::math::Vector3<double>(deltaX, deltaY, deltaZ);
	  msgs::Set(nodePose_msg.add_points(), nodePoints);
	}  
      */
      //publish messages
      this->sbPub->Publish(nodePose_msg);   

    }
//////////////////////////////////////////////////////////////////////////////////////////
     public: btSoftBody* Init_ClothAttach(btSoftBodyWorldInfo* m_softBodyWorldInfo)
    {
    
      //TRACEDEMO
      const btScalar	s=2;
      const btScalar	h=4;
      const int		r=20;
      /*
      btSoftBody* psb=btSoftBodyHelpers::CreatePatch( *m_softBodyWorldInfo, 
						      btVector3(-s,-s,0.5), 
						      btVector3(+s,-s,0.5), 
						      btVector3(-s, s,0.5),
						      btVector3(+s, s,0.5),
						      r, r, 1+2+4+8, true );
      */
      std::ifstream vfs;
      vfs.open("V.txt");
      int vcnt;
      vfs >> vcnt;
      btScalar* vertices = new btScalar[vcnt*3];
      for( int i=0; i<vcnt; i++ ){
	btScalar x, y, z;
	vfs >> x >> z >> y;	
	vertices[i*3+0] = x;
	vertices[i*3+1] = y;
	vertices[i*3+2] = z+0.3;
      }
      vfs.close();

      std::ifstream ffs;
      ffs.open("F.txt");
      int fcnt;
      ffs >> fcnt;
      int* faces = new int[fcnt*3];
      for( int i=0; i<fcnt; i++ ){
	int v1, v2, v3;
	ffs >> v1 >> v2 >> v3;	
	faces[i*3+0] = v1-1;
	faces[i*3+1] = v2-1;
	faces[i*3+2] = v3-1;
      }
      ffs.close();
      /*
      ffs.open("F.txt");
      ffs >> fcnt;
      for( int i=fcnt; i<2*fcnt; i++ ){
	int v1, v2, v3;
	ffs >> v3 >> v2 >> v1;	
	faces[i*3+0] = v1-1;
	faces[i*3+1] = v2-1;
	faces[i*3+2] = v3-1;
      }
      ffs.close();
      */
      btSoftBody* psb=btSoftBodyHelpers::CreateFromTriMesh(*m_softBodyWorldInfo,
							   vertices,
							   faces,
							   fcnt );

      //min distance allowed between softbody and other objects. 
      psb->getCollisionShape()->setMargin(0.001);
      //psb->setTotalMass(0.001, true);
      //psb->setPose( true, false );

      //btSoftBody::Material* pm=psb->appendMaterial();
      btSoftBody::Material* pm=psb->m_materials[0];
      pm->m_kLST                              =       1.0;
      //b->m_cfg.kPR = 0.01;
      pm->m_kAST                              =       1.0;
      pm->m_kVST                              =       1.0;
      pm->m_flags            -=      btSoftBody::fMaterial::DebugDraw;
      psb->generateBendingConstraints(5, pm);
      //std::cout << "Klst: " << pm->m_kLST << std::endl;
      //std::cout << pm << std::endl;

      //psb->setTotalMass(0.001);
      for( int i=0; i<psb->m_nodes.size(); i++ ){
	if( psb->m_nodes[i].m_x.getY() < -0.24 || 
	    psb->m_nodes[i].m_x.getY() > 0.24 )
	  psb->setMass( i, 0.0 );
	else
	  psb->setMass( i, 0.000001 );
      }

      //psb->updateConstants();
      //uses the bullet link to access the physics engine, allowing our new soft body to enter the world
      this->b_link->initSoftBodyPhysics(psb);
      this->b_link->addSoftLinkToWorld(psb);

      return psb;
    }
    

    /////////////////////////////////////////////////////////////////////////////////////////////

    //Pointer to a BulletLink: this will be used as the bridge between bullet and Gazebo, enabling us to access the physics
    //  engine and thus the bullet world. This allows us to add a softbody to the gazebo simulation that can interact with 
    //  other gazebo obects. 
    boost::shared_ptr<gazebo::physics::BulletLink> b_link;
    
    //the softbody
    private: boost::shared_ptr<btSoftBody> softBody;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Node for publishing the SoftBody data
    private: transport::NodePtr node;

    //publisher for fluid visual messages
    private: transport::PublisherPtr sbPub;


  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}


