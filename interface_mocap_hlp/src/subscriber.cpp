
/**
* This node is responsible for the communication between the MoCap and the HLP. 
*/

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher quadPub;

/**
* This is the subscriber callback function
* Whenever MoCap publishes data, it communicates it to the HLP
*/
void quadCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	/**
	* This is the message object containing the data to be communicated between the modules
	*/
	geometry_msgs::PoseWithCovarianceStamped posture;
	posture.header=msg->header;
	posture.pose.pose.position.x=msg->pose.position.x;
	posture.pose.pose.position.y=msg->pose.position.y;
	posture.pose.pose.position.z=msg->pose.position.z;
	posture.pose.pose.orientation.x=msg->pose.orientation.x;
	posture.pose.pose.orientation.y=msg->pose.orientation.y;
	posture.pose.pose.orientation.z=msg->pose.orientation.z;
	posture.pose.pose.orientation.w=msg->pose.orientation.w;
	quadPub.publish(posture);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reader");
	ros::NodeHandle n;
	/**
	* The MoCap publishes the quadrotor data on /Robot_1/pose topic
	*/
	ros::Subscriber quadSub = n.subscribe("Robot_1/pose", 1, quadCallback);
	/**
	* The HLP requires the feedback of the quadrotor on the /fcu/pose topic
	*/
	quadPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("fcu/pose", 1);
	ros::spin();
	return 0;
}
