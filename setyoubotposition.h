// Set youbot position
// Author: Jonathan Hoff

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

using namespace std;
using namespace fcl;

CollisionObject * set_obstacles(CollisionObject obs_arr[12])
{
	// Initialize obstacle objects
	// Geometry of obstacles
	boost::shared_ptr<Box> geom_obs_1(new Box(2.0, 2.0, 1.0));
	boost::shared_ptr<Box> geom_obs_2(new Box(.75, .75, .75));
	boost::shared_ptr<Box> geom_obs_3(new Box(.75, .75, .75));
	boost::shared_ptr<Box> geom_obs_4(new Box(.5, .5, .5));

	boost::shared_ptr<Box> geom_wall_1(new Box(8.0, .5, .5));
	boost::shared_ptr<Box> geom_wall_2(new Box(.5, 8.0, .5));
	boost::shared_ptr<Box> geom_wall_3(new Box(8.0, .5, .5));
	boost::shared_ptr<Box> geom_wall_4(new Box(.5, 8.0, .5));

	boost::shared_ptr<Box> geom_obs_5_1(new Box(.4, .4, .2));
	boost::shared_ptr<Box> geom_obs_5_2(new Box(.05, .05, .15));
	boost::shared_ptr<Box> geom_obs_5_3(new Box(.05, .05, .15));
	boost::shared_ptr<Box> geom_obs_5_4(new Box(.05, .25, .05));

	// Position of obstacles
	Transform3f tf_obs_1, tf_obs_2, tf_obs_3, tf_obs_4;
	Transform3f tf_wall_1, tf_wall_2, tf_wall_3, tf_wall_4;
	Transform3f tf_obs_5_1, tf_obs_5_2, tf_obs_5_3, tf_obs_5_4;

	tf_obs_1.setIdentity();
	tf_obs_2.setIdentity();
	tf_obs_3.setIdentity();
	tf_obs_4.setIdentity();

	tf_wall_1.setIdentity();
	tf_wall_2.setIdentity();
	tf_wall_3.setIdentity();
	tf_wall_4.setIdentity();

	tf_obs_5_1.setIdentity();
	tf_obs_5_2.setIdentity();
	tf_obs_5_3.setIdentity();
	tf_obs_5_4.setIdentity();

	tf_obs_1.setTranslation(Vec3f(2.5, 2.5, 0.5));
	tf_obs_2.setTranslation(Vec3f(5.0, 1.0, .375));
	tf_obs_3.setTranslation(Vec3f(3.0, 5.0, .375));
	tf_obs_4.setTranslation(Vec3f(.0, 3.0, .25));

	tf_wall_1.setTranslation(Vec3f(2.5, -1.25, .25));
	tf_wall_2.setTranslation(Vec3f(6.25, 2.5, .25));
	tf_wall_3.setTranslation(Vec3f(2.5, 6.25, .25));
	tf_wall_4.setTranslation(Vec3f(-1.25, 2.5, .25));

	tf_obs_5_1.setTranslation(Vec3f(.6, .0, .1));
	tf_obs_5_2.setTranslation(Vec3f(.5, .1, .275));
	tf_obs_5_3.setTranslation(Vec3f(.5, -.1, .275));
	tf_obs_5_4.setTranslation(Vec3f(.5, .0, .375));

	CollisionObject obs_1(geom_obs_1, tf_obs_1);
	CollisionObject obs_2(geom_obs_2, tf_obs_2);
	CollisionObject obs_3(geom_obs_3, tf_obs_3);
	CollisionObject obs_4(geom_obs_4, tf_obs_4);

	CollisionObject wall_1(geom_wall_1, tf_wall_1);
	CollisionObject wall_2(geom_wall_2, tf_wall_2);
	CollisionObject wall_3(geom_wall_3, tf_wall_3);
	CollisionObject wall_4(geom_wall_4, tf_wall_4);

	CollisionObject obs_5_1(geom_obs_5_1, tf_obs_5_1);
	CollisionObject obs_5_2(geom_obs_5_2, tf_obs_5_2);
	CollisionObject obs_5_3(geom_obs_5_3, tf_obs_5_3);
	CollisionObject obs_5_4(geom_obs_5_4, tf_obs_5_4);

	// Array of obstacles for checking base of youbot
	//CollisionObject obs_arr [12] = {obs_1,obs_2,obs_3,obs_4,wall_1,wall_2,wall_3,wall_3,obs_5_1,obs_5_2,obs_5_3,obs_5_4};

	//CollisionObject * obs_arr;

	//CollisionObject * obs_arr;
	obs_arr[0] = obs_1;
	obs_arr[1] = obs_2;
	obs_arr[2] = obs_3;
	obs_arr[3] = obs_4;
	obs_arr[4] = wall_1;
	obs_arr[5] = wall_2;
	obs_arr[6] = wall_3;
	obs_arr[7] = wall_4;
	obs_arr[8] = obs_5_1;
	obs_arr[9] = obs_5_2;
	obs_arr[10] = obs_5_3;
	obs_arr[11] = obs_5_4;

	return obs_arr;
}

/*
CollisionObject * arm()
{
// Geometry of youbot arms
boost::shared_ptr<Box> geom_base_comb(new Box(0.5701,0.3570,0.1600)); // base, arm base, and wheels
boost::shared_ptr<Box> geom_arm_1(new Box(0.1697,0.1699,0.1060));
boost::shared_ptr<Box> geom_arm_2(new Box(0.22,0.0745,0.0823));
boost::shared_ptr<Box> geom_arm_3(new Box(0.1920,0.0591,0.0750));
boost::shared_ptr<Box> geom_arm_4_5_6(new Box(0.2401,0.0581,0.0987)); // arm 4 and gripper

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
CollisionObject ybota [5] = {base_comb, arm_1, arm_2, arm_3, arm_4_5_6};
// Array of initial orienations
Transform3f orient_arr [5] = {tf_base_comb, tf_arm_1, tf_arm_2, tf_arm_3, tf_arm_4_5_6};

return ybota;
}
*/

// from given configuration of youbot base, set it as an obstacle
CollisionObject set_youbot_base(float x, float y, float z)
{
	// Youbot base
	// Geometry of youbot base
	boost::shared_ptr<Box> geom_ybotb(new Box(.6, .4, .6));
	Transform3f tf_ybotb;
	tf_ybotb.setIdentity();
	tf_ybotb.setTranslation(Vec3f(x, y, z));
	CollisionObject ybotb(geom_ybotb, tf_ybotb); // initialize
	ybotb.setTranslation(Vec3f(x, y, z));
	return ybotb;
}

// from given configuration of youbot arm, find the forward kinematics and set it as an obstacle
CollisionObject * set_youbot_arm(double jnt_ang[5], CollisionObject ybota[5], Transform3f orient_arr[5])
{
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
	//CollisionObject ybota [5] = {base_comb, arm_1, arm_2, arm_3, arm_4_5_6};
	ybota[0] = base_comb;
	ybota[1] = arm_1;
	ybota[2] = arm_2;
	ybota[3] = arm_3;
	ybota[4] = arm_4_5_6;

	// Array of initial orienations
	//Transform3f orient_arr [5] = {tf_base_comb, tf_arm_1, tf_arm_2, tf_arm_3, tf_arm_4_5_6};
	orient_arr[0] = tf_base_comb;
	orient_arr[1] = tf_arm_1;
	orient_arr[2] = tf_arm_2;
	orient_arr[3] = tf_arm_3;
	orient_arr[4] = tf_arm_4_5_6;

	// "Forward kinematics of KUKA Youbot," Hyongju Park (04/08/2016)
	// See
	// <http://www.youbot-store.com/developers/documentation/kuka-youbot-kinematics-dynamics-and-3d-model>
	// to find more details on kinematics, dynamics and 3D model of KUKA YouBot
	// Transcribed from MATLAB code

	// pose the robot
	double pos_b[2] = { 0, 0 };      // positions (x,y) of the robot
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
	double ang_min[5] = { -169 / 180 * pi, -65 / 180 * pi, -151 / 180 * pi, -102.5 / 180 * pi, -165 / 180 * pi };
	double ang_max[5] = { 169 / 180 * pi, 90 / 180 * pi, 146 / 180 * pi, 102.5 / 180 * pi, 165 / 180 * pi };

	// D-H parameters
	// d:  offset along previous z
	// th: angle about previous z
	// a:  link lengths along old x to new x
	// ap: link twist angles about previous x

	double a[7] = { (143 + 24)*0.001, 33 * 0.001, 155 * 0.001, 135 * 0.001, 113.6*0.001, 0 * 0.001, 0 * 0.001 };
	double ap[7] = { pi, pi / 2, 0, 0, -pi / 2, pi / 2, 0 };
	double d[7] = { (46 + 84 + 115)*0.001, 0 * 0.001, 0 * 0.001, 0 * 0.001, 0 * 0.001, 0 * 0.001, 57.16*0.001 };
	double th[7] = { 0, jnt_ang[0], jnt_ang[1] - pi / 2, jnt_ang[2], jnt_ang[3], pi / 2, jnt_ang[4] + pi / 2 };

	// Homogeneous transformation matrices
	// world - robot
	Matrix3f A_b_rot(cos(th_b), -sin(th_b), 0, sin(th_b), cos(th_b), 0, 0, 0, 1);
	Transform3f A_b(A_b_rot, Vec3f(pos_b[0], pos_b[1], 0));

	Matrix3f A_rot[7];
	Transform3f A[7];
	Vec3f A_trans[7];
	for (int i = 0; i<7; i++)
	{
		A_rot[i] = Matrix3f(cos(th[i]), -sin(th[i])*cos(ap[i]), sin(th[i])*sin(ap[i]), sin(th[i]), cos(th[i])*cos(ap[i]), -cos(th[i])*sin(ap[i]), 0, sin(ap[i]), cos(ap[i]));
		A_trans[i] = Vec3f(a[i] * cos(th[i]), a[i] * sin(th[i]), d[i]);
		A[i] = Transform3f(A_rot[i], A_trans[i]);
	}

	Transform3f T01 = A_b*A[0];     // world - base frame
	Transform3f T02 = T01*A[1];     // world - arm base frame
	Transform3f T03 = T02*A[2];     // world - arm joint 1
	Transform3f T04 = T03*A[3];     // world - arm joint 2
	Transform3f T05 = T04*A[4];     // world - arm joint 3
	Transform3f T06 = T05*A[5];     // world - arm joint 4
	Transform3f T07 = T06*A[6];     // world - arm joint 5

	Transform3f rot_arm[5];
	rot_arm[0] = Transform3f(Matrix3f(cos(th[1]), -sin(th[1]), 0, sin(th[1]), cos(th[1]), 0, 0, 0, 1)); // arm 1
	rot_arm[1] = Transform3f(Matrix3f(cos(th[2]), -sin(th[2]), 0, sin(th[2]), cos(th[2]), 0, 0, 0, 1)); // arm 2
	rot_arm[2] = Transform3f(Matrix3f(cos(th[3]), -sin(th[3]), 0, sin(th[3]), cos(th[3]), 0, 0, 0, 1)); // arm 3
	rot_arm[3] = Transform3f(Matrix3f(cos(th[4]), -sin(th[4]), 0, sin(th[4]), cos(th[4]), 0, 0, 0, 1)); // arm 4
	rot_arm[4] = Transform3f(Matrix3f(cos(th[5]), -sin(th[5]), 0, sin(th[5]), cos(th[5]), 0, 0, 0, 1)); // arm 5

	tf_base_comb = A_b*orient_arr[0]; // arm base
	tf_arm_1 = T01*rot_arm[0] * orient_arr[1]; // arm 1
	tf_arm_2 = T02*rot_arm[1] * orient_arr[2]; // arm 2
	tf_arm_3 = T03*rot_arm[2] * orient_arr[3]; // arm 3
	tf_arm_4_5_6 = T04*rot_arm[3] * orient_arr[4]; // arm 4 (arm_4_5_6)
	//tf_arm_5 = T05*rot_arm[4]*orient_arr[5]; // arm 5

	//Transform3f tf_base_comb = A_b*orient_arr[0]; // arm base
	//Transform3f tf_arm_1 = T01*rot_arm[0]*orient_arr[1]; // arm 1
	//Transform3f tf_arm_2 = T02*rot_arm[1]*orient_arr[2]; // arm 2
	//Transform3f tf_arm_3 = T03*rot_arm[2]*orient_arr[3]; // arm 3
	//Transform3f tf_arm_4_5_6 = T04*rot_arm[3]*orient_arr[4]; // arm 4 (arm_4_5_6)
	////Transform3f tf_arm_5 = T05*rot_arm[4]*orient_arr[5]; // arm 5


	ybota[0].setTransform(tf_base_comb);
	ybota[1].setTransform(tf_arm_1);
	ybota[2].setTransform(tf_arm_2);
	ybota[3].setTransform(tf_arm_3);
	ybota[4].setTransform(tf_arm_4_5_6);

	return ybota;
}

