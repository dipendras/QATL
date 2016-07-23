#include "ros/ros.h"
#include "std_msgs/Time.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
/*
float plat_prev_time=0;
float plat_prev_x=0;
float plat_prev_y=0;
float plat_prev_z=0;
float diff_time = 0;
*/
ros::Publisher quadPub;

//ros::Publisher platformPub;

void quadCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
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
/*
void platformCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Twist velo;

	diff_time=(ros::Time::now().toSec())-plat_prev_time;

	if (diff_time!=0)
	{	
		velo.linear.x=((msg->pose.position.x)-plat_prev_x)/diff_time;
		velo.linear.y=((msg->pose.position.y)-plat_prev_y)/diff_time;
		velo.linear.z=((msg->pose.position.z)-plat_prev_z)/diff_time;
	}

	plat_prev_time=ros::Time::now().toSec();
	plat_prev_x=msg->pose.position.x;
	plat_prev_y=msg->pose.position.y;
	plat_prev_z=msg->pose.position.z;

	platformPub.publish(velo);
}
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "reader");

	ros::NodeHandle n;

//	plat_prev_time=ros::Time::now().toSec();
	
	ros::Subscriber quadSub = n.subscribe("Robot_1/pose", 1, quadCallback);

//	ros::Subscriber platformSub = n.subscribe("Robot_2/pose", 1, platformCallback);

	quadPub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("fcu/pose", 1);

//	platformPub = n.advertise<geometry_msgs::Twist>("Platform_Velocities", 1);
	  
	ros::spin();

	return 0;
}
