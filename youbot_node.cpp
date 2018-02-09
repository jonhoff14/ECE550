// Author: Jonathan Hoff

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <nav_msgs/Odometry.h> 
#include <cmath>
#include <string>
#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/collision.h>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <list>

#include "structures.h"
#include "robotconfig.h"
#include "RRTrobot.h"
#include "RRTmanipulator.h"
#include "manipulatorconfig.h"

using namespace std;
using namespace fcl;

class YoubotClass
{
public:
	// Constructor
	YoubotClass(ros::NodeHandle& n);

	// Destructor
	~YoubotClass(){}

	// callback function to read odometry of youbot
	void odomCallback(const nav_msgs::Odometry& msg);
	void joint_statesCallback(const sensor_msgs::JointState& msg);

	// set new desired position of youbot
	trajectory_msgs::JointTrajectory move_youbot_arm(double jnt_ang_goal[5]);
	// Gradient of the potiential of goal
	geometry_msgs::Twist move_youbot_base(double x0, double y0, double x1, double y1);

	/*
	// return true if collision
	bool check_collision(CollisionObject co1, CollisionObject co2);
	// return true if path has collision
	bool check_path_base(double x0, double y0, double x1, double y1, CollisionObject ybotb, CollisionObject obs_arr [12]);
	bool check_path_arm(double jnt_ang0 [5], double jnt_ang1 [5], CollisionObject ybota [5], Transform3f orient_arr [5], CollisionObject obs_arr [12]);
	// from current configuration of youbot base, set it as an obstacle
	CollisionObject set_youbot_base(double x, double y, double z, CollisionObject ybotb);
	CollisionObject * set_youbot_arm(double jnt_ang [5], CollisionObject ybota [5], Transform3f orient_arr [5]);
	*/

	// Member variables

	// Odometry variables
	//seq;
	double x_pos;
	double y_pos;
	double z_pos;
	double w;
	double x_vel;
	double y_vel;
	double z_ang;

	// Sensor variables for arm
	double jnt_ang_sens[5];

	double x_goal;
	double y_goal;

	double x_path[3]; // x coordinates for youbot base to follow
	double y_path[3]; // y coordinates for youbot base to follows

	ros::Publisher pub_base;
	ros::Publisher pub_arm;
	ros::Subscriber sub_base;
	ros::Subscriber sub_arm;
};

YoubotClass::YoubotClass(ros::NodeHandle& n) {
	const double pi = boost::math::constants::pi<double>();
	x_pos = 0.0;
	y_pos = 0.0;
	z_pos = 0.0;
	w = 0.0;
	x_vel = 0.0;
	y_vel = 0.0;
	z_ang = 0.0;


	x_goal = 0.0;
	y_goal = 0.0;

	//cout <<  "Enter input goal x coordinate: ";
	//cin >> x_goal;
	//cout <<  "Enter input goal y coordinate: ";
	//cin >> y_goal;

	pub_base = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	pub_arm = n.advertise<trajectory_msgs::JointTrajectory>("arm_1/arm_controller/command", 1000);

	sub_base = n.subscribe("odom", 1000, &YoubotClass::odomCallback, this);
	sub_arm = n.subscribe("joint_states", 1000, &YoubotClass::joint_statesCallback, this);

	//ros::spin(); //  enters a loop pumping callback functions until CTRL + C
	ros::spinOnce(); // listen to messages
	ros::Rate loop_rate(5);

	// Youbot base
	// Geometry of youbot base
	boost::shared_ptr<Box> geom_ybotb(new Box(.6, .4, .6));
	Transform3f tf_ybotb;
	tf_ybotb.setIdentity();
	tf_ybotb.setTranslation(Vec3f(x_pos, y_pos, z_pos));
	CollisionObject ybotb(geom_ybotb, tf_ybotb); // initialize

	// Geometry of youbot arms
	boost::shared_ptr<Box> geom_base_comb(new Box(0.5701, 0.3570, 0.1600)); // base, arm base, and wheels
	boost::shared_ptr<Box> geom_arm_1(new Box(0.1697, 0.1699, 0.1060));
	boost::shared_ptr<Box> geom_arm_2(new Box(0.22, 0.0745, 0.0823));
	boost::shared_ptr<Box> geom_arm_3(new Box(0.1920, 0.0591, 0.0750));
	boost::shared_ptr<Box> geom_arm_4_5_6(new Box(0.2401, 0.0581, 0.0987)); // arm 4 and gripper

	// Position of arms
	Transform3f tf_base_comb, tf_arm_1, tf_arm_2, tf_arm_3, tf_arm_4_5_6;

	tf_base_comb.setIdentity();
	tf_arm_1.setIdentity();
	tf_arm_2.setIdentity();
	tf_arm_3.setIdentity();
	tf_arm_4_5_6.setIdentity();

	tf_base_comb.setTranslation(Vec3f(-0.0014, 0.0, 0.0956));
	tf_arm_1.setTranslation(Vec3f(0.0, 0.0, 0.159));
	tf_arm_2.setTranslation(Vec3f(0.0740, 0.0, -0.0412));
	tf_arm_3.setTranslation(Vec3f(0.0662, 0.0, 0.0370));
	tf_arm_4_5_6.setTranslation(Vec3f(0.0869, 0.0, 0.0));

	CollisionObject base_comb(geom_base_comb, tf_base_comb);
	CollisionObject arm_1(geom_arm_1, tf_arm_1);
	CollisionObject arm_2(geom_arm_2, tf_arm_2);
	CollisionObject arm_3(geom_arm_3, tf_arm_3);
	CollisionObject arm_4_5_6(geom_arm_4_5_6, tf_arm_4_5_6);

	// Array of obstacles of arm
	CollisionObject ybota[5] = { base_comb, arm_1, arm_2, arm_3, arm_4_5_6 };
	// Array of initial orienations
	Transform3f orient_arr[5] = { tf_base_comb, tf_arm_1, tf_arm_2, tf_arm_3, tf_arm_4_5_6 };

	CollisionObject obs_arr_temp[12];
	CollisionObject * obs_arr;
	obs_arr = set_obstacles(obs_arr_temp);


	/*
	boost::shared_ptr<Box> data(new Box(1,1,1));
	Transform3f tf0, tf1;
	tf0.setIdentity();
	tf0.setTranslation(Vec3f(0.51,0,0));
	tf0 = tf0*tf0;
	tf0.setQuatRotation(Quaternion3f(.6, .8, 0, 0));
	tf1.setIdentity();

	CollisionObject ob1(data, tf0);
	CollisionObject ob2(data, tf1);
	bool res;
	res = YoubotClass::check_collision(ob1, ob2);
	cout << "Result of collision is " << res << endl;
	*/


	//bool res;
	//res = YoubotClass::check_collision(ob1, ob2);
	//cout << "Result of collision is " << res << endl;

	// Insert RRT here---------------------
	// Random path
	float x_path[2] = { 2.5, 2.5 }; // x coordinates for youbot base to follow
	float y_path[2] = { 2.5, 2.5 }; // y coordinates for youbot base to follow

	bool pathclear;
	pathclear = check_path_base(x_path[0], y_path[0], x_path[1], y_path[1], obs_arr);
	cout << "Base path is: " << pathclear << endl;




	double jnt_ang_start[5] = { 0, 0, 0, 0, 0 };
	double jnt_ang_goal[5] = { 0, 0.628, 1.964, -1.047, 0 };
	//double jnt_ang_goal[5] = {0,pi/2,0,0,0};
	pathclear = check_path_arm(jnt_ang_start, jnt_ang_goal, ybota, orient_arr, obs_arr);

	cout << "Arm path is: " << pathclear << endl;

	//cout << check_collision(ybota[0], ybota[1]) << endl;



	//---------------------------------------------------

	bool choice = false; // true for base, false for arm
	list<robot_config>::iterator itr;
	list<manipulator_config>::iterator itr_manip;
	list<robot_config> robot_path;
	list<manipulator_config> robot_path_manip;

	double pos_error;
	bool collision_occured;
	if (choice)
	{
		///*
		robot_config q_start;

		q_start.set(0, 0, 0, 0);

		robot_config q_goal;
		q_goal.set(5, 5, -1, -1);

		//  Mat image
		RRT_robot planner_robot(q_start, 500, 3);

		planner_robot.plan_path();
		cout << "Done" << endl;

		robot_path = planner_robot.get_path(q_goal);


		cout << "Size of path: " << robot_path.size() << endl;
	}
	else
	{
		///*
		manipulator_config q_start;

		q_start.set(0, 0, 0, 0, 0, 0);

		manipulator_config q_goal;
		q_goal.set(0, 0.628, 1.964, -1.047, -1, -1);

		//  Mat image
		RRT_manipulator planner_robot(q_start, 2500, 0.001);

		planner_robot.plan_path();
		cout << "Done" << endl;

		robot_path_manip = planner_robot.get_path(q_goal);


		cout << "Size of path: " << robot_path_manip.size() << endl;
		//*/
	}


	itr = robot_path.begin();


	//*/
	//---------------------------------------------------
	// Move youbot arm
	trajectory_msgs::JointTrajectory arm_pos;
	// Move youbot base
	geometry_msgs::Twist vel;

	while (ros::ok())
	{
		ros::spinOnce(); // listen to messages

		if (choice)
		{
			///*
			// Keep updating bounding box of youbot base
			ybotb = set_youbot_base(x_pos, y_pos, z_pos);

			// Move youbot base using RRT
			vel = move_youbot_base(x_pos, y_pos, (*itr).get_x(), (*itr).get_y());
			pub_base.publish(vel);

			collision_occured = check_path_base(x_pos, y_pos, x_pos, y_pos, obs_arr);
			pos_error = sqrt(pow(x_pos - (*itr).get_x(), 2) + pow(y_pos - (*itr).get_y(), 2));

			//cout << pos_error << endl;
			//cout << collision_occured << endl;
			if (pos_error<0.05 && std::distance(robot_path.begin(), itr) < robot_path.size() - 1)
			{
				advance(itr, 1);
				//cout << "x_pos is: " << x_pos << endl;
				//cout << "y_pos is: " << y_pos << endl;
				//cout << "x_goal is: " << (*itr).get_x() << endl;
				//cout << "y_goal is: " << (*itr).get_y() << endl;
			}
			//*/


		}
		else
		{
			/*
			arm_pos = move_youbot_arm(jnt_ang_goal); // set new desired position of arm
			pub_arm.publish(arm_pos);

			cout << "Arm joint 1 position: " << jnt_ang_sens[0] << endl;
			cout << "Arm joint 2 position: " << jnt_ang_sens[1] << endl;
			cout << "Arm joint 3 position: " << jnt_ang_sens[2] << endl;
			cout << "Arm joint 4 position: " << jnt_ang_sens[3] << endl;
			cout << "Arm joint 5 position: " << jnt_ang_sens[4] << endl;

			cout << "jnt_ang_goal 3 is: " << jnt_ang_goal[2] << endl;

			//CollisionObject *set_arm; // need pointer to return array
			//set_arm = set_youbot_arm(jnt_ang_goal, ybota, orient_arr);
			*/
		}

		loop_rate.sleep();
	}
}

void YoubotClass::odomCallback(const nav_msgs::Odometry& msg)
{
	//seq = msg.header.seq;
	this->x_pos = msg.pose.pose.position.x;
	this->y_pos = msg.pose.pose.position.y;
	this->z_pos = msg.pose.pose.position.z;
	this->w = msg.pose.pose.orientation.w;
	this->x_vel = msg.twist.twist.linear.x;
	this->y_vel = msg.twist.twist.linear.y;
	this->z_ang = msg.twist.twist.angular.z;
	//cout << "xpos: " << x_pos <<" ypos: " << y_pos << endl;
	//ROS_INFO_STREAM("I received " << "position=(" << x_pos << ", " << y_pos << "), " << "orientation(RAD)=" << z_ang);
}

void YoubotClass::joint_statesCallback(const sensor_msgs::JointState& msg)
{
	this->jnt_ang_sens[0] = msg.position[0];
	this->jnt_ang_sens[1] = msg.position[1];
	this->jnt_ang_sens[2] = msg.position[2];
	this->jnt_ang_sens[3] = msg.position[3];
	this->jnt_ang_sens[4] = msg.position[4];
}

trajectory_msgs::JointTrajectory YoubotClass::move_youbot_arm(double jnt_ang_goal[5])
{
	/*
	{header: {stamp: {secs: 0,nsecs: 0}}
	joint_names: ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
	points:
	positions:  [2.95, 1.13, -2.55, 1.79, 2.879]
	velocities: []
	accelerations: [],
	time_from_start: {secs: 3, nsecs: 0}}]}
	*/

	//our goal variable
	trajectory_msgs::JointTrajectory goal;

	//goal.header.frame_id = "base_footprint";
	goal.header.stamp = ros::Time::now();// + ros::Duration(1.0);

	// First, the joint names, which apply to all waypoints
	//[arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5]
	goal.joint_names.push_back("arm_joint_1");
	goal.joint_names.push_back("arm_joint_2");
	goal.joint_names.push_back("arm_joint_3");
	goal.joint_names.push_back("arm_joint_4");
	goal.joint_names.push_back("arm_joint_5");

	// We will have two waypoints in this goal trajectory
	goal.points.resize(1);

	// First trajectory point
	// Positions
	int ind = 0;
	//goal.points[ind].positions.resize(5);
	//goal.points[ind].positions[0] = 0.0;
	//goal.points[ind].positions[1] = 0.0;
	//goal.points[ind].positions[2] = 0.0;
	//goal.points[ind].positions[3] = 0.0;
	//goal.points[ind].positions[4] = 0.0;
	// Velocities
	//goal.points[ind].velocities.resize(5);
	//for (size_t j = 0; j < 5; ++j)
	//{
	//	goal.points[ind].velocities[j] = 0.0;
	//}
	// To be reached 1 second after starting along the trajectory
	//goal.points[ind].time_from_start = ros::Duration(1.0);

	// Second trajectory point
	// Positions
	//ind += 1;

	//for (int i=0;i<5;i++){cout << "joint " << i << "is: " << jnt_ang_goal[i] << endl;}

	goal.points[ind].positions.resize(5);
	goal.points[ind].positions[0] = jnt_ang_goal[0] + 2.95;
	goal.points[ind].positions[1] = jnt_ang_goal[1] + 1.13;
	goal.points[ind].positions[2] = jnt_ang_goal[2] - 2.55;
	goal.points[ind].positions[3] = jnt_ang_goal[3] + 1.79;
	goal.points[ind].positions[4] = jnt_ang_goal[4] + 2.879;
	// Velocities
	//goal.points[ind].velocities.resize(5);
	//for (size_t j = 0; j < 5; ++j)
	//{
	//	goal.points[ind].velocities[j] = 0.0;
	//}

	// To be reached 3 seconds after starting along the trajectory

	goal.points[ind].time_from_start = ros::Duration(3.0);


	//we are done; return the goal
	return goal;
}


geometry_msgs::Twist YoubotClass::move_youbot_base(double x0, double y0, double x1, double y1) // Gradient of the potiential of goal
{
	double d_star = 2; // switching point between conical and quadratic
	double zeta = 1; // gain of potential
	double dx = x1 - x0;
	double dy = y1 - y0;
	double dist = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2));

	geometry_msgs::Twist vel;
	if (dist <= d_star)
	{
		// Quadratic gradient
		vel.linear.x = zeta*dx;
		vel.linear.y = zeta*dy;
	}
	else if (dist > d_star)
	{
		// Conical gradient
		vel.linear.x = d_star*zeta*dx / dist;
		vel.linear.y = d_star*zeta*dy / dist;
	}

	return vel;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "youbot_node");
	ros::NodeHandle n;
	YoubotClass yb(n);
	return 0;
}





/*
bool YoubotClass::check_collision(CollisionObject co1, CollisionObject co2)
{

static const int num_max_contacts = std::numeric_limits<int>::max();
static const bool enable_contact = true;
fcl::CollisionResult result;
fcl::CollisionRequest request(num_max_contacts,enable_contact);

fcl::collide(&co1, &co2, request, result);
vector<Contact> contacts;
result.getContacts(contacts);

if (contacts.size() > 0)
{
//cout << contacts.size() << " contacts found" << endl;
return true;
}
else
return false;
}


bool YoubotClass::check_path_base(double x0, double y0, double x1, double y1, CollisionObject ybotb, CollisionObject obs_arr [12])
{
double path_length = sqrt(pow((x0-x1),2)+pow((y0-y1),2));
double num_steps = round(path_length/0.1);

//cout << num_steps << endl;
double x;
double y;
bool col = false; // true for collision
double lambda;
CollisionObject ybotb_temp; // temporary storage of youbot bounding box
for (int i = 0; i < num_steps; i++)
{
lambda = (i+1)/num_steps;
x = (1-lambda)*x0+lambda*x1; // interpolate along path
y = (1-lambda)*y0+lambda*y1; // interpolate along path
//cout << "x is: " << x << endl;
//cout << "y is: " << y << endl;
ybotb_temp = set_youbot_base(x,y,0,ybotb);
for (int j = 0; j < 12; j++)
{
if (!col) // only check if there hasn't been a collision
col = check_collision(ybotb_temp, obs_arr[j]); // Check each obstacle for collision
else // immediatly end if collision is detected
return col; // return true if collision
}
}
// Return false if collision free
return col;
}


bool YoubotClass::check_path_arm(double jnt_ang0 [5], double jnt_ang1 [5], CollisionObject ybota [5], Transform3f orient_arr [5], CollisionObject obs_arr [12])
{
double path_length = sqrt(pow((jnt_ang0[0]-jnt_ang1[0]),2)+pow((jnt_ang0[1]-jnt_ang1[1]),2)+pow((jnt_ang0[2]-jnt_ang1[2]),2)+pow((jnt_ang0[3]-jnt_ang1[3]),2)+pow((jnt_ang0[4]-jnt_ang1[4]),2));
double num_steps = round(path_length/0.05);
//cout << "pathlength is: " << path_length << ", number of steps is: " << num_steps << endl;
//cout << num_steps << endl;
double jnt_ang [5];

bool col = false; // true for collision
double lambda;

CollisionObject *ybota_temp; // temporary storage of youbot bounding boxes, need pointer to return array

// Check for collisions all along path
for (int i = 0; i < num_steps; i++)
{
// Interpolate each joint angle at given step
lambda = (i+1)/num_steps;
for (int k = 0; k < 5; k++) {jnt_ang[k] = (1-lambda)*jnt_ang0[k]+lambda*jnt_ang1[k];}

//cout << "jnt_ang[0] is: " << jnt_ang[0] << endl;
//cout << "jnt_ang[1] is: " << jnt_ang[1] << endl;

ybota_temp = set_youbot_arm(jnt_ang, ybota, orient_arr); // apply FWD kinematics

for (int j0 = 0; j0 < 5; j0++)
{
for (int j1 = 8; j1 < 12; j1++) // only check last 4 obstacles
{
if (!col) // only check if there hasn't been a collision
col = check_collision(ybota_temp[j0], obs_arr[j1]); // Check each obstacle for collision
else // immediatly end if collision is detected
return col; // return true if collision
}
}
}

// Return false if collision free
return col;
}

*/
/*
CollisionObject YoubotClass::set_youbot_base(double x, double y, double z, CollisionObject ybotb)
{
ybotb.setTranslation(Vec3f(x,y,z));
return ybotb;
}

CollisionObject * YoubotClass::set_youbot_arm(double jnt_ang [5], CollisionObject ybota [5], Transform3f orient_arr [5])
{
// "Forward kinematics of KUKA Youbot," Hyongju Park (04/08/2016)
// See
// <http://www.youbot-store.com/developers/documentation/kuka-youbot-kinematics-dynamics-and-3d-model>
// to find more details on kinematics, dynamics and 3D model of KUKA YouBot
// Transcribed from MATLAB code

// pose the robot
double pos_b [2] = {0,0};      // positions (x,y) of the robot
double th_b = 0;           // orientation (yaw) of the robot

// these are the angles when arm is streched vertically
// arm joint 1, arm joint 2, arm joint 3, arm joint 4, arm joint 5
// double jnt_ang [5]; // configuration

// For the youbot model in ROS, the following offset angles must be added to the actual joint angles
//double jnt_offset [5] = {2.95, 1.13, -2.55, 1.79, 2.879}; // DONT USE HERE THOUGH, not needed for FWD Kinematics
// thus the joint angles to be used in ROS is:
//double jnt_ang_ROS [5];
//for (int i=0; i<5; i++) {jnt_ang_ROS[i] = jnt_ang[i] + jnt_offset[i];}

// min and max of joint angles
const double pi = boost::math::constants::pi<double>();
double ang_min [5] = {-169/180*pi, -65/180*pi, -151/180*pi, -102.5/180*pi, -165/180*pi};
double ang_max [5] = {169/180*pi, 90/180*pi, 146/180*pi, 102.5/180*pi, 165/180*pi};

// D-H parameters
// d:  offset along previous z
// th: angle about previous z
// a:  link lengths along old x to new x
// ap: link twist angles about previous x

double a [7] = {(143+24)*0.001, 33*0.001, 155*0.001, 135*0.001, 113.6*0.001, 0*0.001, 0*0.001};
double ap [7] = {pi, pi/2, 0, 0, -pi/2, pi/2, 0};
double d [7] = {(46+84+115)*0.001, 0*0.001, 0*0.001, 0*0.001, 0*0.001, 0*0.001, 57.16*0.001};
double th [7] = {0, jnt_ang[0], jnt_ang[1]-pi/2, jnt_ang[2], jnt_ang[3], pi/2, jnt_ang[4]+pi/2};

// Homogeneous transformation matrices
// world - robot
Matrix3f A_b_rot(cos(th_b),-sin(th_b),0,sin(th_b),cos(th_b),0,0,0,1);
Transform3f A_b (A_b_rot,Vec3f(pos_b[0],pos_b[1],0));

Matrix3f A_rot[7];
Transform3f A[7];
Vec3f A_trans[7];
for (int i=0;i<7;i++)
{
A_rot[i] = Matrix3f(cos(th[i]), -sin(th[i])*cos(ap[i]), sin(th[i])*sin(ap[i]),sin(th[i]), cos(th[i])*cos(ap[i]), -cos(th[i])*sin(ap[i]),0, sin(ap[i]), cos(ap[i]));
A_trans[i] = Vec3f(a[i]*cos(th[i]),a[i]*sin(th[i]),d[i]);
A[i] = Transform3f(A_rot[i],A_trans[i]);
}

Transform3f T01 = A_b*A[0];     // world - base frame
Transform3f T02 = T01*A[1];     // world - arm base frame
Transform3f T03 = T02*A[2];     // world - arm joint 1
Transform3f T04 = T03*A[3];     // world - arm joint 2
Transform3f T05 = T04*A[4];     // world - arm joint 3
Transform3f T06 = T05*A[5];     // world - arm joint 4
Transform3f T07 = T06*A[6];     // world - arm joint 5

Transform3f rot_arm [5];
rot_arm[0] = Transform3f(Matrix3f(cos(th[1]), -sin(th[1]), 0, sin(th[1]), cos(th[1]), 0, 0, 0, 1)); // arm 1
rot_arm[1] = Transform3f(Matrix3f(cos(th[2]), -sin(th[2]), 0, sin(th[2]), cos(th[2]), 0, 0, 0, 1)); // arm 2
rot_arm[2] = Transform3f(Matrix3f(cos(th[3]), -sin(th[3]), 0, sin(th[3]), cos(th[3]), 0, 0, 0, 1)); // arm 3
rot_arm[3] = Transform3f(Matrix3f(cos(th[4]), -sin(th[4]), 0, sin(th[4]), cos(th[4]), 0, 0, 0, 1)); // arm 4
rot_arm[4] = Transform3f(Matrix3f(cos(th[5]), -sin(th[5]), 0, sin(th[5]), cos(th[5]), 0, 0, 0, 1)); // arm 5

Transform3f tf_base_comb = A_b*orient_arr[0]; // arm base
Transform3f tf_arm_1 = T01*rot_arm[0]*orient_arr[1]; // arm 1
Transform3f tf_arm_2 = T02*rot_arm[1]*orient_arr[2]; // arm 2
Transform3f tf_arm_3 = T03*rot_arm[2]*orient_arr[3]; // arm 3
Transform3f tf_arm_4_5_6 = T04*rot_arm[3]*orient_arr[4]; // arm 4 (arm_4_5_6)
//Transform3f tf_arm_5 = T05*rot_arm[4]*orient_arr[5]; // arm 5

ybota[0].setTransform(tf_base_comb);
ybota[1].setTransform(tf_arm_1);
ybota[2].setTransform(tf_arm_2);
ybota[3].setTransform(tf_arm_3);
ybota[4].setTransform(tf_arm_4_5_6);

return ybota;
}

*/


/*
//boost::shared_ptr<Box> box0(new Box(1,1,1));
//boost::shared_ptr<Box> box1(new Box(1,1,1));
//res = YoubotClass::check_collision(box0, box1);
bool YoubotClass::check_collision(boost::shared_ptr<Box> box0, boost::shared_ptr<Box> box1)
{
GJKSolver_libccd solver;
Vec3f contact_points;
FCL_REAL penetration_depth;
Vec3f normal;

Transform3f tf0, tf1;
tf0.setIdentity();
tf0.setTranslation(Vec3f(1.0,0,0));
//tf0.setQuatRotation(Quaternion3f(.6, .8, 0, 0));
tf1.setIdentity();


bool res = solver.shapeIntersect(*box0, tf0, *box1, tf1, &contact_points, &penetration_depth, &normal);

cout << "contact points: " << contact_points << endl;
cout << "pen depth: " << penetration_depth << endl;
cout << "normal: " << normal << endl;
cout << "result: " << res << endl;

static const int num_max_contacts = std::numeric_limits<int>::max();
static const bool enable_contact = true;
fcl::CollisionResult result;
fcl::CollisionRequest request(num_max_contacts,enable_contact);

CollisionObject co0(box0, tf0);
CollisionObject co1(box1, tf1);

fcl::collide(&co0, &co1, request, result);
vector<Contact> contacts;
result.getContacts(contacts);

cout << contacts.size() << " contacts found" << endl;
//for(const Contact &contact : contacts) {
//cout << "position: " << contact.pos << endl;
//}

return res;
}
*/

