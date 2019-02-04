#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>
#include <cmath>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  /// A plugin to control a Robot Arm.
  class MyarmPlugin : public ModelPlugin
  {
    /// Constructor
    public: MyarmPlugin() {}

    /// The load function is called by Gazebo when the plugin is inserted into simulation
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      printf("Plugin is Loaded\n");
      // Store the model pointer for convenience.
      this->model = _model;

      // Safety check, model must have at least one joint.
      if(_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, no joints available.\n";
        return;
      }

      // Get the first joint. We are making an assumption about the model having at least one joint.
      std::cerr << "Total number of joints : " << _model->GetJointCount() << "\n";

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyarmPlugin::OnUpdate, this));
      //store all Joints for later use
      physics::Joint_V jointVector = this->model->GetJoints();
       
      // Setup a PID-controller.
      this->pid = common::PID(3000,0,0);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
         int argc = 0;
	 	 char **argv = NULL;
	 	 ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      //create Topics and Subscribe tos them
      for(physics::Joint_V::iterator jit=jointVector.begin(); jit!=jointVector.end(); ++jit)
      {
         // if revolute joint. if not, ignore joint
         if((*jit)->GetType() != 576)
	 	 continue;    
	 std::string subPath = "/" + (*jit)->GetName() + "/setPosition";
	 // Create topics for SETTING POSITION
	 ros::SubscribeOptions so = 
		 ros::SubscribeOptions::create<std_msgs::Float32>(
		      subPath,1,boost::bind(&MyarmPlugin::AngleCallback, this, _1, (*jit)->GetName()),
		      ros::VoidPtr(), &this->rosQueue);
	 // add subscriber to list
         this->rosSubList.push_back(this->rosNode->subscribe(so));
	 std::cerr << "Topic name: " << subPath << "\n";
         joints.push_back((*jit)->GetName());

	 // save the Position of joint in a map
	 #if GAZEBO_MAJOR_VERSION < 9
           jointAngles[(*jit)->GetName()] = (*jit)->GetAngle(0).Radian();
	 #else
	   jointAngles[(*jit)->GetName()] = (*jit)->Position(0);
	 #endif	 
      }

      //create topic for keyboardlistener
      ros::SubscribeOptions so2 = 
		 ros::SubscribeOptions::create<std_msgs::Int32>(
		      "/keyboard_input",1,boost::bind(&MyarmPlugin::KeyboardCallback, this, _1), 
		      ros::VoidPtr(), &this->keyboardQueue);
      this->rosSubList.push_back(this->rosNode->subscribe(so2));
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&MyarmPlugin::QueueThread, this));
      this->keyboardQueueThread = std::thread(std::bind(&MyarmPlugin::KeyboardQueueThread, this));
    }

    //  ROS helper function that processes messages
    private: void QueueThread()
    {
       static const double timeout = 0.01;
       while (this->rosNode->ok())
       {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
       }
    }
    // take value from message and write it to map
    private: void AngleCallback(const std_msgs::Float32ConstPtr &_msg, std::string jointName)
    {
	float angle = _msg->data;
	float newAngle = (angle*M_PI)/180;
	// you update the position of where you want the joint to move
	this->jointAngles[jointName] = newAngle;
	std::cerr << "Angle:" << newAngle << "\n";
        return;
    }
    //  ROS helper function that processes messages
    private: void KeyboardQueueThread()
    {
       static const double timeout = 0.01;
       while (this->rosNode->ok())
       {
          this->keyboardQueue.callAvailable(ros::WallDuration(timeout));
       }
    }
    // keyboardcallback
    private: void KeyboardCallback(const std_msgs::Int32ConstPtr &_msg)
    {
	int f = _msg->data;
	// how many degrees a single buttonpress moves the joint
	float angle = 3;
	switch (f)
	{
	   // q : GrossesGelenk +
	   case 113:
		this->jointAngles["GrossesGelenkLink"] = this->jointAngles["GrossesGelenkLink"] + (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["GrossesGelenkLink"] << "\n";
	   	break; 

	   // a : GrossesGelenk -
	   case 97:	
		this->jointAngles["GrossesGelenkLink"] = this->jointAngles["GrossesGelenkLink"] - (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["GrossesGelenkLink"] << "\n";
	   	break; 

	   // w :  MittleresGelenk +
	   case 119:	
		this->jointAngles["MittleresGelenkLink"] = this->jointAngles["MittleresGelenkLink"] + (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["MittleresGelenkLink"] << "\n";
	   	break; 

	   // s : MittleresGelenk -
	   case 115:
		this->jointAngles["MittleresGelenkLink"] = this->jointAngles["MittleresGelenkLink"] - (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["MittleresGelenkLink"] << "\n";
	   	break; 

	   // e : KleinesGelenk +
	   case 101:
		this->jointAngles["KleinesGelenkLink"] = this->jointAngles["KleinesGelenkLink"] + (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["KleinesGelenkLink"] << "\n";
	   	break; 

	   // d : KleinesGelenk -
	   case 100:	
		this->jointAngles["KleinesGelenkLink"] = this->jointAngles["KleinesGelenkLink"] - (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["KleinesGelenkLink"] << "\n";
	   	break;	
	   // y : Gross_Bodenplatte -
	   case 121:	
		this->jointAngles["Gross_Bodenplatte"] = this->jointAngles["Gross_Bodenplatte"] - (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["Gross_Bodenplatte"] << "\n";
	   	break;
	  // x : Gross_Bodenplatte +
	   case 120:	
		this->jointAngles["Gross_Bodenplatte"] = this->jointAngles["Gross_Bodenplatte"] + (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["Gross_Bodenplatte"] << "\n";
	   	break;
	  // c : Mittel_Oberschwinge -
	   case 99:	
		this->jointAngles["Mittel_Oberschwinge"] = this->jointAngles["Mittel_Oberschwinge"] - (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["Mittel_Oberschwinge"] << "\n";
	   	break;
	   // v : Mittel_Oberschwinge +
	   case 118:	
		this->jointAngles["Mittel_Oberschwinge"] = this->jointAngles["Mittel_Oberschwinge"] + (angle*M_PI)/180;
		std::cerr << "Angle:" << this->jointAngles["Mittel_Oberschwinge"] << "\n";
	   	break;
	   // default is empty
	   default: break;
	}
	return;
    }
    // Gets called, everytime the world is updated
    private: void OnUpdate()
    {
      // compute the steptime for the PID
      #if GAZEBO_MAJOR_VERSION < 9
      common::Time currTime = this->model->GetWorld()->GetSimTime();
      #else
      common::Time currTime = this->model->GetWorld()->SimTime();
      #endif
      common::Time stepTime = currTime - this->prevUpdateTime;
      this->prevUpdateTime = currTime;

      for(auto it=this->joints.begin(); it!=joints.end(); ++it)
      {		
	// set the current position of the joint, the target position 
	// and the maximum effort limit
	double pos_target = this->jointAngles[(*it)];
	#if GAZEBO_MAJOR_VERSION < 9
	double pos_curr = this->model->GetJoint(*it)->GetAngle(0).Radian();
	#else
	double pos_curr = this->model->GetJoint(*it)->Position(0);
	#endif
	double max_cmd = this->joint_max_effort;

	// calculate the error between the current position and the target one
	double pos_err = pos_curr - pos_target;

	// compute the effort via the PID, which you will apply on the joint
	double effort_cmd = this->pid.Update(pos_err, stepTime);

	// check if the effort is larger than the maximum permitted one
	effort_cmd = effort_cmd > max_cmd ? max_cmd :
		    (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);

	// apply the force on the joint
	this->model->GetJoint(*it)->SetForce(0, effort_cmd);
      }
      return;
    }

    ///  Pointer to the model.
    private: physics::ModelPtr model;
    ///  Pointer to the joint.
    private: physics::JointPtr joint;
    ///  3 PID controllers for the 3 joints.
    private: common::PID pid;

    // Pointer for UpdateEvent in Gazebo
    private: event::ConnectionPtr updateConnection;
    private: common::Time prevUpdateTime;
    
    private: float joint_max_effort = 300;
    //List with all subscribers
    private: std::list<ros::Subscriber> rosSubList;
    //List with all revolute joint names
    private: std::list<std::string> joints;
    //map with joints' angles by their names
    private: std::map<std::string, double> jointAngles;

    //  A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    //  A ROS subscriber for SETTING JOINTS POSITION
    private: ros::Subscriber rosSub;
    //  A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    private: ros::CallbackQueue keyboardQueue;
    //  A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    private: std::thread keyboardQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyarmPlugin)
}
