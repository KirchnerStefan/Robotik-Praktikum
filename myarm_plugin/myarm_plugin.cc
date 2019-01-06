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

      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyarmPlugin::OnUpdate, this));
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyarmPlugin::OnUpdate, this));
      //store all Joints for later use
      physics::Joint_V jointVector = this->model->GetJoints();
       
      // Setup a PID-controller.
      this->pid = common::PID(100,0,0.1);
      //this->pidSmall = common::PID(5,0.1,0.1,5,-5);
      //this->pidMiddle = common::PID(40,2.5,10,5,-5);
      //this->pidBig = common::PID(50,2,10,5,-5);

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
		      subPath,1,boost::bind(&MyarmPlugin::ROSCallback, this, _1, (*jit)->GetName()),
		      ros::VoidPtr(), &this->rosQueue);
	 // add subscriber to list
         this->rosSubList.push_back(this->rosNode->subscribe(so));
	 std::cerr << "Topic name: " << subPath << "\n";
         joints.push_back((*jit)->GetName());

	 // save the Position of joint in a map
	 #if GAZEBO_MAJOR_VERSION < 9
         jointAngles[(*jit)->GetName()] = (*jit)->GetAngle(0).Radian();
	 std::cerr << "Target angle of" << (*jit) << " is " << (*jit)->GetAngle(0).Radian() << "\n";
	 #else
	 jointAngles[(*jit)->GetName()] = (*jit)->Position(0);
	 std::cerr << "Target angle of" << (*jit)->GetName() << " is " << (*jit)->Position(0) << "\n";
	 #endif	 
      }
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&MyarmPlugin::QueueThread, this));
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
    private: void ROSCallback(const std_msgs::Float32ConstPtr &_msg, std::string jointName)
    {
	float angle = _msg->data;
	float newAngle = (angle*M_PI)/180;
	// you update the position of where you want the joint to move
	this->jointAngles[jointName] = newAngle;
	std::cerr << "Angle:" << newAngle << "\n";
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
    //private: common::PID pidSmall;
    //private: common::PID pidMiddle;
    //private: common::PID pidBig;

    // Pointer for UpdateEvent in Gazebo
    private: event::ConnectionPtr updateConnection;
    private: common::Time prevUpdateTime;
    
    //private: double joint_target_pos; //replaced with map
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
    //  A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyarmPlugin)
}
