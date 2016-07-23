/**
*
* This node implements the tracking and landing module. 
* The quadrotor flies to some initial position and starts tracking the platform until it is within tolerance limits and then it lands.  
* Safety features are also incorporated in this node.
*
* The implemented control equation for tracking can be used for three types of control laws by simply setting some paramenters.
*
* Settings for the different control laws:
* 1) PD Controller
*     - set ki and kv to zero
*     - kp and kd are the position error and velocity error gains respectively to be tuned 
* 2) PID Controller
*     - set kv to zero
*     - kp, ki and kd are the position error, sum of position error and velocity error gains respectively to be tuned 
* 3) Static Feedback Controller
*     - set ki and kd to zero
*     - set kv to one
*     - kp is the position error gain to be tuned 
*
* The mav_ctrl message is used to give commands to the qaudrotor
* The type variable in the message determines the type of control instruction
*   - type = 2 is for the velocity control
*   - type = 3 is for the position control
*
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <asctec_hl_comm/WaypointAction.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <geometry_msgs/PoseStamped.h>

/**
* List of the available parameters for the module
*/
//! Arena Safety Limits
float arenaLimitX=1.5;
float arenaLimitY=1.5;
float arenaLimitZmin=0.3;
float arenaLimitZmax=2.0;
//! Height to which the quadrotor initially flies before tracking starts 
float initialHeight=1.0;
//! Maximum allowable speed for the quadrotor
float maxSpeed=0.35;
float accuracy=0.2;
//! Minimum clearance, the fallback height and the boolean flag in case clearance limit is exceeded 
float clearanceHeight=-0.27;
float fallbackHeight=0.75;
bool flag=false;
//! The position of the mobile platform
float x_desired=0;
float y_desired=0;
float z_desired=0.3;
//! The velocity of the mobile platform
float x_dot_desired=0;
float y_dot_desired=0;
float z_dot_desired=0;
//! The position of the quadrotor
float x_actual=0;
float y_actual=0;
float z_actual=0;
//! The velocity of the quadrotor
float x_dot_actual=0;
float y_dot_actual=0;
float z_dot_actual=0;
//! The controller outputs
float x_dot_output=0;
float y_dot_output=0;
float z_dot_output=0;
//! The control equation gains
float kp=1.4;
float ki=0;
float kd=0;
float kv=1;
//! Frequency of the control outputs
float freq=25;
//! Tolerance limit for landing
float tol=0.04;

/**
* This is the callback function for the platform position feedback
*/
void platformCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x_desired=msg->pose.position.x;
	y_desired=msg->pose.position.y;
	z_desired=msg->pose.position.z;
}

/**
* This is the callback function for the quadrotor position feedback
*/
void quadCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x_actual=msg->pose.position.x;
	y_actual=msg->pose.position.y;
	z_actual=msg->pose.position.z;
}

/**
* This is the callback function for the asctec waypoint feedback
*/
void feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr & fb)
{
  const geometry_msgs::Point32 & wp = fb->current_pos;
  ROS_INFO("got feedback: %fm %fm %fm %f° ", wp.x, wp.y, wp.z, fb->current_yaw*180/M_PI);
}

/**
* This is the callback function for the asctec waypoint server
*/
void activeCb()
{
  ROS_INFO("Goal just went active");
}

/**
* This is the callback function for the asctec waypoint completion task
*/
void doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result)
{
  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  }
  else
  {
    ROS_WARN("Finished in state [%s]", state.toString().c_str());
  }
  const geometry_msgs::Point32 & wp = result->result_pos;
  ROS_INFO("Reached waypoint: %fm %fm %fm %f°",wp.x, wp.y, wp.z, result->result_yaw*180/M_PI);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "asctec_track_land");
  ros::NodeHandle nh;

  //! The timout for the waypoint server tasks is 10s
  ros::Duration timeout(10);
  
  //! Creating the waypoint action client object which uses the /fcu/waypoint topic
  actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> ac(nh, "fcu/waypoint", true);

  /**
  * Publishers for the control output and the positions nd velocity errors
  * Control output is on the /fcu/control topic
  * Position error is on the /pos_error topic
  * Velocity error is on the /vel_error topic
  */
  ros::Publisher publ = nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1);
  ros::Publisher errorPosPub = nh.advertise<geometry_msgs::PoseStamped>("pos_error",1);
  ros::Publisher errorVelPub = nh.advertise<geometry_msgs::PoseStamped>("vel_error",1);

  /**
  * Subscribers for the position feedbacks
  * Position feedback of the quadrotor is on the /Robot_1/pose topic
  * Position feedback of the mobile platform is on the /Robot_2/pose topic
  */
  ros::Subscriber quadSub = nh.subscribe("Robot_1/pose", 1, quadCallback);
  ros::Subscriber platformSub = nh.subscribe("Robot_2/pose", 1, platformCallback);

  //! The goal position for the asctec waypoint server
  asctec_hl_comm::WaypointGoal goal;

  /**
  * The asctec waypoint server needs to be started before this node starts executing
  * In case the server has not been started, it waits for it to start
  */
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  //! Setting the initial fly position through the waypoinr server
  goal.goal_pos.x = 0;
  goal.goal_pos.y = 0;
  goal.goal_pos.z = initialHeight;
  goal.goal_yaw = 0;
  goal.max_speed.x = maxSpeed;
  goal.max_speed.y = maxSpeed;
  goal.max_speed.z = maxSpeed;
  goal.accuracy_position = accuracy;
  goal.accuracy_orientation = 0;
  goal.timeout = timeout.toSec();

  //! Send the action to the server
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCB);

  //! Wait for the action to return
  bool finished_before_timeout = ac.waitForResult(timeout);

  //! Check for the completion of the action within timeout limit
  if (!finished_before_timeout)
  {
    ROS_WARN("Action did not finish before the time out.");
  }

  //! Variables for calculating the velocity of quadrotor and mobile platform
  float xp_prev=x_desired;
  float yp_prev=y_desired;
  float xq_prev=x_actual;
  float yq_prev=y_actual;

  //! Object creation for the gain errors and the control output
  geometry_msgs::PoseStamped errorP;
  geometry_msgs::PoseStamped errorV;
  geometry_msgs::PoseStamped errorS;
  asctec_hl_comm::mav_ctrl output;

  //! Initializatio of the summation of the position error
  errorS.pose.position.x=0;
  errorS.pose.position.y=0;

  //! Update after the set duration
  ros::Rate rate(freq);

  /**
  * Loop for the tracking module
  * It only exits if the arena safety limits have been violated
  */
  while(1)
  {
    ros::spinOnce();

	//! Calculating the position error
    errorP.pose.position.x=x_desired-x_actual;
    errorP.pose.position.y=y_desired-y_actual;
    errorP.pose.position.z=z_desired-z_actual;
    errorPosPub.publish(errorP);

	//! Calculating the velocity error
    errorV.pose.position.x=x_dot_desired-x_dot_actual;
    errorV.pose.position.y=y_dot_desired-y_dot_actual;
    errorVelPub.publish(errorV);

	//! Calculating the sum of the position errors
    errorS.pose.position.x+=errorP.pose.position.x;
    errorS.pose.position.y+=errorP.pose.position.y;

	//! Check for the arena limit violation
    if (x_actual<arenaLimitX && x_actual>-arenaLimitX  && y_actual<arenaLimitY && y_actual>-arenaLimitY && z_actual<arenaLimitZmax && z_actual>arenaLimitZmin)
    {

	  //! Calculating the velocity of the mobile platform
      x_dot_desired=(x_desired-xp_prev)*freq;
      y_dot_desired=(y_desired-yp_prev)*freq;
      xp_prev=x_desired;
      yp_prev=y_desired;

	  //! Calculating the velocity of the quadrotor
      x_dot_actual=(x_actual-xq_prev)*freq;
      y_dot_actual=(y_actual-yq_prev)*freq;
      xq_prev=x_actual;
      yq_prev=y_actual;

	  //! Calculating the control outputs
      output.type=2;
      output.x = kv*(x_dot_desired) + kd*(errorV.pose.position.x) + kp*(errorP.pose.position.x) + ki*(errorS.pose.position.x);
      output.y = kv*(y_dot_desired) + kd*(errorV.pose.position.y) + kp*(errorP.pose.position.y) + ki*(errorS.pose.position.y);
      output.z=0;
      output.yaw=0;
      output.v_max_xy=maxSpeed;
      output.v_max_z=maxSpeed;

	  //! Limiting the maximum speed of motion
      if (output.x>maxSpeed)
      {
        output.x=maxSpeed;
      }
      if (output.x<-maxSpeed)
      {
        output.x=-maxSpeed;
      }
      if (output.y>maxSpeed)
      {
        output.y=maxSpeed;
      }
      if (output.y<-maxSpeed)
      {
        output.y=-maxSpeed;
      }

	  //! Checking whether landing is possible
      if (((y_desired-y_actual)<tol) && ((y_desired-y_actual)>-tol) && ((x_desired-x_actual)<tol) && ((x_desired-x_actual)>-tol))
      {
        output.z=-0.1;
      } 
      else
      {
        output.z=0;
      }

	  //! Check for flying within safe height
      if ((z_desired-z_actual)>clearanceHeight)
      {
        flag=true;
      }

	  //! Fallback height flying in case flying at a dangerous level  
      if (z_actual<fallbackHeight && flag)
      {
        output.x=0;
        output.y=0;
        output.z=0.2;
      }
      else
      {
        flag=false;
      }

	  //! Displaying position and velocity of the quadrotor and platform to user
      ROS_INFO("QUAD OUTPUT = [%f,%f,%f] PLAT VELO = [%f,%f] QUAD POS = [%f,%f,%f] PLAT POS = [%f,%f,%f] SUM ERROR = [%f,%f] POS ERROR = [%f,%f]",output.x,output.y,output.z,x_dot_desired,y_dot_desired,x_actual,y_actual,z_actual,x_desired,y_desired,z_desired,errorS.pose.position.x,errorS.pose.position.y,errorP.pose.position.x,errorP.pose.position.y);
	  
	  //! Publishing the control outputs
      publ.publish(output);

	  //! Wait till next time period
      rate.sleep();
    }
    else
    {
      break;
    }
  }

  //! User prompt indicating safety limits have been violated
  ROS_INFO("Arena Safety Limits Exceeded \nLanding At Starting Position\n");

  //! When arena safe limit exceeded fly back to arena centre
  output.type=3;
  output.x=0;
  output.y=0;
  output.z=1;
  output.yaw=0;
  output.v_max_xy=maxSpeed;
  output.v_max_z=maxSpeed;
  publ.publish(output);
  ros::Duration(8).sleep();

  //! Landing
  output.type=2;
  output.x=0;
  output.y=0;
  output.z=-0.1;
  output.yaw=0;
  output.v_max_xy=0.1;
  output.v_max_z=0.1;
  publ.publish(output);
  ros::Duration(10).sleep();

  return 0;
}
