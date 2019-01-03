#include <ros/ros.h>
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("", 1000);
  //Wiederhole mit 1 Hz
  ros::Rate loop_rate(1);

  float i = 0.0;
  bool rechts = false;

  while (ros::ok())
  {
    std_msgs::Float32 msg;
	// solange rechts nicht gesetzt 
	while(rechts == false)
	{	
		// gehe nach links
		i -= 0.01;	
		// sende wert via topic
		msg.data = i;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// Wenn links angekommen
		if(i == -1.7)
		{	//setze rechts
			rechts = true;	
		}
	}
	// solange rechts gesetzt
	while(rechts == true)
	{	
		// gehe nach rechts
		i += 0.01;
		// sende Wert via topic
		msg.data = i;
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		// Wenn rechts angekommen
		if(i == 1.7)
		{	// setze rechts nicht
			rechts = false;	
		}
	}
  }

  return 0;
}
