/*
Description: A visual plugin for allowing the interaction of a mesh with a softbody.

Author: Hudson Thomas
Date: July 25, 2016
*/


#include <boost/bind.hpp>
#include <gazebo/common/common.hh>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <Ogre.h>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/common/Events.hh"
#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>


namespace gazebo
{
  class Visual_Plugin : public VisualPlugin
  {
  public: Visual_Plugin(){ this-> newMsgRecieved = false; }
  public: ~Visual_Plugin(){ }
  public: void Load( 	rendering::VisualPtr  	_visual,
			sdf::ElementPtr  	_sdf 
			){
    initGraphics(); 
}

    ///////////////////////////////////////////////////////////////////////////////////////////////////
  public: void Init()
  {  
    //initialize communication
    this->node = transport::NodePtr(new transport::Node());  
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&Visual_Plugin::InitAtPreRenderEvent, this ) );
    this->node->Init(); 
    this->testSub = this->node->Subscribe("~/sb_pose", &Visual_Plugin::OnPoseMsg, this);
  }
      
    ///////////////////////////////////////////////////////////////////////////////////////////////////

  public: void InitAtPreRenderEvent()
  {

    if(newMsgRecieved)
      {
	// get the mesh of the entity                                                                     
	Ogre::MeshPtr pMesh = entity->getMesh();

	// get number of vertices of the mesh                                                             
	int nMaxVert=pMesh->sharedVertexData->vertexCount ;

	// get vertex elements by positions                                                               
	const Ogre::VertexElement* VertexEle_POS =
	  pMesh->sharedVertexData->vertexDeclaration->findElementBySemantic( Ogre::VES_POSITION );
	
	// get the vertex position buffer                                                                 
	Ogre::HardwareVertexBufferSharedPtr VertexBufPOS =
	  pMesh->sharedVertexData->vertexBufferBinding->getBuffer
	  ( VertexEle_POS->getSource() );
	
	// get a pointer to the shared buffer                                                             
	Ogre::HardwareVertexBuffer* hvb = VertexBufPOS.getPointer();
	
	// lock the buffer for reading/writing                                                            
	unsigned char* VertexPtrPOS =
	  static_cast<unsigned char*>
	  ( hvb->lock( Ogre::HardwareVertexBuffer::HBL_NORMAL ) );
	
	// get size in of the vertices                                                                    
	int VertSizePOS=VertexBufPOS->getVertexSize();
	
	// this will get the vertex                                                                       
	float * pElementPOS=NULL ;
	if( !this->pose.empty() ){
	  // for all vertices                                                                               
	  for(int nVert=0 ; nVert<nMaxVert; nVert++){
	    
	    // get the vertex data pointed by vertex pointer                                                
	    VertexEle_POS->baseVertexPointerToElement(VertexPtrPOS, &pElementPOS );
	    // change the position (weird that z is [1])     
	    pElementPOS[0]= this->pose.at(nVert).at(0); 
	    pElementPOS[1]= this->pose.at(nVert).at(1);
	    pElementPOS[2]= this->pose.at(nVert).at(2);
	    // increase the vertex pointer                                                                  
	    VertexPtrPOS+=VertSizePOS;
	  }
	}
	// unlock the buffer                                                                              
	hvb->unlock();
	this->newMsgRecieved = false;

      }
  }
    ///////////////////////////////////////////////////////////////////////////////////////////////////

  public: void OnPoseMsg(ConstPointCloudPtr &_msg)
    {
      if(!newMsgRecieved){

	this->pose.clear();
	int i;
	for(i = 0; i<_msg->points_size();i++)
	  {
	    std::vector<double> inner;
	    inner.push_back(_msg->points(i).x());
	    inner.push_back(_msg->points(i).y());
	    inner.push_back(_msg->points(i).z());
	    this->pose.push_back(inner);
	  } 
	this->newMsgRecieved = true;

      }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////

  public: void initGraphics(){

#define IDX(_x_,_y_)    ((_y_)*rx+(_x_))

    //TRACEDEMO
    const btScalar	s=2;
    const btScalar	h=4;
    const int		r=20, rx=20, ry=20;
    btSoftBodyWorldInfo worldInfo = btSoftBodyWorldInfo();

    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");
    
    /// Create one submesh
    Ogre::SubMesh* sub = msh->createSubMesh();
    
    const float sqrt13 = 0.577350269f; /* sqrt(1/3) */
 
    /// Define the vertices (8 vertices, each have 3 floats for position and 3 for normal)
    std::ifstream vfs, nfs;
    vfs.open("V.txt");
    int nVertices;
    vfs >> nVertices;
    int vbufCount = 3*2*nVertices;
    
    float vertices[vbufCount];
    for( int i=0; i<nVertices; i++ ){
      float x, y, z;
      vfs >> x >> y >> z;	
      vertices[i*6+0] = x;
      vertices[i*6+1] = y;
      vertices[i*6+2] = z+0.3;

      //nfs >> x >> y >> z;	
      //float d = sqrt( x*x + y*y + z*z );
      vertices[i*6+3] = cos( atan2( z, x ) );
      vertices[i*6+4] = 0.0;
      vertices[i*6+5] = sin( atan2( z, x ) );

    }
    vfs.close();

    Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();
    Ogre::RGBA colours[nVertices];
    Ogre::RGBA *pColour = colours;
    for( int i=0; i<nVertices; i++ ){
      rs->convertColourValue(Ogre::ColourValue(0.3,0.3,0.3), pColour++);
    }
    // Use render system to convert colour value since colour packing varies
    //rs->convertColourValue(Ogre::ColourValue(1.0,0.0,0.0), pColour++); //0 colour
    //rs->convertColourValue(Ogre::ColourValue(1.0,1.0,0.0), pColour++); //1 colour
    //rs->convertColourValue(Ogre::ColourValue(0.0,1.0,0.0), pColour++); //2 colour
    //rs->convertColourValue(Ogre::ColourValue(0.0,0.0,0.0), pColour++); //3 colour
    //rs->convertColourValue(Ogre::ColourValue(1.0,0.0,1.0), pColour++); //4 colour
    //rs->convertColourValue(Ogre::ColourValue(1.0,1.0,1.0), pColour++); //5 colour
    //rs->convertColourValue(Ogre::ColourValue(0.0,1.0,1.0), pColour++); //6 colour
    //rs->convertColourValue(Ogre::ColourValue(0.0,0.0,1.0), pColour++); //7 colour
 
    std::ifstream ffs;
    ffs.open("F.txt");
    int fcnt;
    ffs >> fcnt;
    unsigned short faces[fcnt*3];
    const size_t ibufCount = fcnt*3;
 
    for( int i=0; i<fcnt; i++ ){
      int v1, v2, v3;
      ffs >> v1 >> v2 >> v3;	
      faces[i*3+0] = v1-1;
      faces[i*3+1] = v2-1;
      faces[i*3+2] = v3-1;
    }
    ffs.close();

 
    /// Create vertex data structure for 8 vertices shared between submeshes
    msh->sharedVertexData = new Ogre::VertexData();
    msh->sharedVertexData->vertexCount = nVertices;
 
    /// Create declaration (memory format) of vertex data
    Ogre::VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount) 
    /// and bytes per vertex (offset)
    Ogre::HardwareVertexBufferSharedPtr vbuf = 
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset,
								     msh->sharedVertexData->vertexCount,
								     Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);
 
    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding; 
    bind->setBinding(0, vbuf);
 
    // 2nd buffer
    offset = 0;
    decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount) 
    /// and bytes per vertex (offset)
    vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset,
									  msh->sharedVertexData->vertexCount,
									  Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);
 
    /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
    bind->setBinding(1, vbuf);
 
    /// Allocate index buffer of the requested number of vertices (ibufCount) 
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
        createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, 
			  ibufCount, 
			  Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
 
    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);
 
    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = ibufCount;
    sub->indexData->indexStart = 0;
 
    /// Set bounding information (for culling)
    msh->_setBounds(Ogre::AxisAlignedBox(-100,-100,-100,100,100,100));
    msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3*100*100));
 
    /// Notify -Mesh object that it has been loaded
    msh->load();

    //initialize scene Manager and Root node
    this->root= Ogre::Root::getSingletonPtr();
    Ogre::SceneManagerEnumerator::SceneManagerIterator smi = root->getSceneManagerIterator();
    Ogre::SceneManagerEnumerator::Instances::iterator it;
    for( it=smi.begin(); it!=smi.end(); it++ ){
      this->smgr = it->second;
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Test/ColourTest", 
									      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT);

    //new stuff
    material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
    // Ogre::TexturePtr mTex = Ogre::TextureManager::getSingleton().load("MLI_texture.png","mli_tex");
    //material->getTechnique(0)->getPass(0)->createTextureUnitState()->setTextureName("mli_tex");


    this->entity = smgr->createEntity("cc", "ColourCube");
    this->entity->setMaterialName("Test/ColourTest");
    Ogre::SceneNode* thisSceneNode = smgr->getRootSceneNode()->createChildSceneNode();
    thisSceneNode->setPosition(0, 0, 0);
    thisSceneNode->attachObject(this->entity);

  
   
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////

  //subscriber node declaration
  private: transport::NodePtr node;
  private: transport::SubscriberPtr testSub;
 
  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  // pointer to the OGRE root node of the gazebo simulation
  private: Ogre::Root* root;

  // pointer to the OGRE scene manager of the gazebo simulation
  private: Ogre::SceneManager* smgr;

  // pointer to the OGRE entity that the mesh visualization will be attached to
  private: Ogre::Entity* entity;
    
  // pointer to a 2d vector containing the 3d position of each node.  
  private: std::vector<std::vector<double>> pose; 

  // timer to make sure that the mesh isn't rendered until the message has been fully recieved. 
  private: bool newMsgRecieved; 
};
// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(Visual_Plugin)
}
