#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <thread>

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
      this->joint = this->model->GetJoints()[0];

	  std::cerr << "Selected Joint: " << this->joint->GetScopedName() << "\n";
       
      // Setup a PID-controller.
      this->pid = common::PID(10,0.1,0.01,10,-10);

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
         int argc = 0;
		 char **argv = NULL;
		 ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      // Create ROS node.
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create topic for SETTING POSITION
      //ros::SubscribeOptions so =
	    //ros::SubscribeOptions::create<std_msgs::Float32>(
	      //"/" + this->model->GetName() + "/setPosition",1,
	      //boost::bind(&MyarmPlugin::OnRosMsg_setPosition, this, _1),
	      //ros::VoidPtr(), &this->rosQueue);
      //this->rosSub = this->rosNode->subscribe(so);
      // Spin up the queue helper thread.
      //this->rosQueueThread = std::thread(std::bind(&MyarmPlugin::QueueThread, this));

	  // subscribing to the rost topic, and calling the callback function
      this->rosSub = this->rosNode->subscribe("/ROS_topic", 10, &MyarmPlugin::ROSCallback, this);
	  // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyarmPlugin::OnUpdate, this));
    }

    // Set the position of the joint via PID
    public: void SetPosition(const double &_pos)
    {
       this->model->GetJointController()->SetPositionTarget(
	    this->joint->GetScopedName(), _pos);
    }
    //  Handle an incoming message from ROS for setting the position
    public: void OnRosMsg_setPosition(const std_msgs::Float32ConstPtr &_msg)
    {
       this->SetPosition(_msg->data);
    }

    //  ROS helper function that processes messages for SETTING POSITION
    private: void QueueThread()
    {
       static const double timeout = 0.01;
       while (this->rosNode->ok())
       {
          this->rosQueue.callAvailable(ros::WallDuration(timeout));
       }
    }

	void ROSCallback(const std_msgs::Float32ConstPtr &_msg)
	{
		 // you update the position of where you want the joint to move
		 this->joint_target_pos = _msg->data;
		 std::cerr << "entered Callback. Target pos is:"<< this->joint_target_pos << "\n";
	}
	void OnUpdate()
	{
		// compute the steptime for the PID
		common::Time currTime = this->model->GetWorld()->GetSimTime();
		common::Time stepTime = currTime - this->prevUpdateTime;
	 	this->prevUpdateTime = currTime;

	    // set the current position of the joint, and the target position, 
		// and the maximum effort limit
		double pos_target = this->joint_target_pos;
		double pos_curr = this->joint->GetAngle(0).Radian();
		double max_cmd = this->joint_max_effort;

		// calculate the error between the current position and the target one
		double pos_err = pos_curr - pos_target;

		// compute the effort via the PID, which you will apply on the joint
		double effort_cmd = this->pid.Update(pos_err, stepTime);

		// check if the effort is larger than the maximum permitted one
		effort_cmd = effort_cmd > max_cmd ? max_cmd :
		            (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);

		// apply the force on the joint
		this->joint->SetForce(0, effort_cmd);
	}

    ///  Pointer to the model.
    private: physics::ModelPtr model;
    ///  Pointer to the joint.
    private: physics::JointPtr joint;
    ///  A PID controller for the joint.
    private: common::PID pid;
	//
	private: event::ConnectionPtr updateConnection;
	private: common::Time prevUpdateTime;
	private: double joint_target_pos;
	private: float joint_max_effort =10;

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
