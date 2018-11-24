#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
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
      this->SelectJoint(0);
       
      // Setup a PID-controller.
      this->pid = common::PID(0.1, 0, 0);

      // Apply PID-Controller for position
      this->model->GetJointController()->SetPositionPID(this->joint->GetScopedName(), this->pid);
      // Default to position zero
      double angle = 0;    
      this->SetPosition(angle);

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
      ros::SubscribeOptions so =
	    ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/" + this->model->GetName() + "/setPosition",1,
	      boost::bind(&MyarmPlugin::OnRosMsg_setPosition, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&MyarmPlugin::QueueThread, this));

      // Create topic for SELECTING JOINT
      ros::SubscribeOptions so2 =
	    ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/" + this->model->GetName() + "/selectJoint",1,
	      boost::bind(&MyarmPlugin::OnRosMsg_selectJoint, this, _1),
	      ros::VoidPtr(), &this->rosQueue2);
       this->rosSub2 = this->rosNode->subscribe(so2);
       // Spin up the queue helper thread.
       this->rosQueueThread2 = std::thread(std::bind(&MyarmPlugin::QueueThread2, this));

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


    // Select Joint that will be altered
    public: void SelectJoint(const int &_jointNumber)
    {      
       this->joint = model->GetJoints()[_jointNumber];
       // Give name of selected Joint
       std::cerr << "Selected joints name is: " << joint->GetScopedName() <<"\n";
    }

// NUR EINE MESSAGE SENDEN, SONST TOPIC VERSTOPFT!!!

    //  Handle an incoming message from ROS for selecting the joint
    public: void OnRosMsg_selectJoint(const std_msgs::Float32ConstPtr &_msg)
    {
       this->SelectJoint(_msg->data);
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
    //  ROS helper function that processes messages for SELECTING JOINT
    private: void QueueThread2()
    {
       static const double timeout = 0.01;
       while (this->rosNode->ok())
       {
          this->rosQueue2.callAvailable(ros::WallDuration(timeout));
       }
    }

    ///  Pointer to the model.
    private: physics::ModelPtr model;

    ///  Pointer to the joint.
    private: physics::JointPtr joint;

    ///  A PID controller for the joint.
    private: common::PID pid;

    //  A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    //  A ROS subscriber for SETTING JOINTS POSITION
    private: ros::Subscriber rosSub;
    //  A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    //  A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    //  A ROS subscriber for SELECTING JOINT
    private: ros::Subscriber rosSub2;
    //  A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    //  A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(MyarmPlugin)
}
