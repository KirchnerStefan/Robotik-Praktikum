#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <gazebo/gazebo.hh>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include <memory>
#include <map>

namespace gazebo
{
    class MyarmPlugin : public ModelPlugin
    {
    	public: MyarmPlugin() {}

        //read every joint of model and initialize ROS topics
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            printf("Plugin is Loaded\n");
      		// Store the model pointer for convenience.
      		this->model = _parent;

      		// Safety check, model must have at least one joint.
      		if(model->GetJointCount() == 0)
      		{
        		std::cerr << "Invalid joint count, no joints available.\n";
        		return;
      		}
            this->updateConnection = event::Events::ConnectWorldUpdateBegin
						(boost::bind(&MyarmPlugin::OnUpdate, this, _1));
            physics::Joint_V jointVector = this->model->GetJoints();

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("MyarmPlugin"));

            // Create a named topic, and subscribe to it for every joint
            for(physics::Joint_V::iterator jit=jointVector.begin(); jit!=jointVector.end(); ++jit)
            {
                //test if revolute joint. if not, ignore joint
                if((*jit)->GetType() != 576)
                    continue;
				// create subscriber
                std::string subPath = (*jit)->GetName();
                ros::SubscribeOptions so = 
					ros::SubscribeOptions::create<std_msgs::Float32>(subPath,
                    1,boost::bind(&MyarmPlugin::OnRosMsg, this, _1, 
					(*jit)->GetName()), ros::VoidPtr(), &this->rosQueue);
				// add subscriber to list
                this->rosSubList.push_back(this->rosNode->subscribe(so));
                joints.push_back((*jit)->GetName());
				std::cerr << "Topic: " << (*jit)->GetName() << "\n";
				// save the Position of joint in a map
				#if GAZEBO_MAJOR_VERSION < 9
                jointAngles[(*jit)->GetName()] = (*jit)->GetAngle(0).Radian();
				#else
				jointAngles[(*jit)->GetName()] = (*jit)->Position(0);
				#endif
            }

            // Spin up the queue helper thread
            this->rosQueueThread = std::thread(std::bind(&MyarmPlugin::QueueThread, this));
        }

        //Handle an incoming message from ROS
        void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, std::string jointName)
        {
            this->jointAngles[jointName] = _msg->data;
            return;
        }

		// keep the joints locked in respectiove positions for every iteration of simulation
        void OnUpdate(const common::UpdateInfo & _info)
        {
            //set velocity and force to zero for every saved joint
            //and set angle to saved value
            for(auto it=this->joints.begin(); it!=joints.end(); ++it)
            {
                this->model->GetJoint(*it)->SetVelocity(0, 0);
                this->model->GetJoint(*it)->SetForce(0, 0);
                this->model->GetJoint(*it)->SetPosition(0, jointAngles[*it]);
            }
            return;
        }

        //ROS helper function that processes messages
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

		// Pointer to the model
        physics::ModelPtr model;
		// Connection
        event::ConnectionPtr updateConnection;
		// ROS Node for Transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        //List with all subscribers
        std::list<ros::Subscriber> rosSubList;

        //List with all revolute joint names
        std::list<std::string> joints;
		//Callbackqueue for processing messages
        ros::CallbackQueue rosQueue;
		//Thread that keeps queue running
        std::thread rosQueueThread;

        //map with joints' angles by their names
        std::map<std::string, double> jointAngles;
    };

    GZ_REGISTER_MODEL_PLUGIN(MyarmPlugin)
}
