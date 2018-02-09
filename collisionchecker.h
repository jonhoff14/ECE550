// Collision checking
// Author: Jonathan Hoff

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h> 
#include <cmath>
#include <string>
#include <iostream>

#include <fcl/shape/geometric_shapes.h>
#include <fcl/shape/geometric_shapes_utility.h>
#include <fcl/narrowphase/narrowphase.h>
#include <fcl/collision.h>


#include "setyoubotposition.h"

using namespace std;
using namespace fcl;

// return true if collision
bool check_collision(CollisionObject co1, CollisionObject co2)
{

	static const int num_max_contacts = std::numeric_limits<int>::max();
	static const bool enable_contact = true;
	fcl::CollisionResult result;
	fcl::CollisionRequest request(num_max_contacts, enable_contact);

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

// return true if path has collision
bool check_path_base(float x0, float y0, float x1, float y1, CollisionObject obs_arr[12])
{
	float path_length = sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2));
	float num_steps = round(path_length / 0.1);

	//cout << num_steps << endl;
	float x;
	float y;
	bool col = false; // true for collision
	float lambda;
	CollisionObject ybotb_temp; // temporary storage of youbot bounding box

	// Check generated sample (if no path)
	if (num_steps == 0) // i.e. if (x0 == x1 && y0 == y1)
	{
		ybotb_temp = set_youbot_base(x0, y0, 0);
		//Vec3f temp = obs_arr[0].getTranslation();
		//cout << temp.data[0] << endl;
		//cout << temp.data[1] << endl;
		//cout << temp.data[2] << endl;

		for (int j = 0; j < 12; j++)
		{
			if (!col) // only check if there hasn't been a collision
				col = check_collision(ybotb_temp, obs_arr[j]); // Check each obstacle for collision
			else // immediatly end if collision is detected
				return col; // return true if collision
		}
	}
	else // Check entire path
	{
		for (int i = 0; i < num_steps; i++)
		{
			lambda = (i + 1) / num_steps;
			x = (1 - lambda)*x0 + lambda*x1; // interpolate along path
			y = (1 - lambda)*y0 + lambda*y1; // interpolate along path
			//cout << "x is: " << x << endl;
			//cout << "y is: " << y << endl;
			ybotb_temp = set_youbot_base(x, y, 0);
			for (int j = 0; j < 12; j++)
			{
				if (!col) // only check if there hasn't been a collision
					col = check_collision(ybotb_temp, obs_arr[j]); // Check each obstacle for collision
				else // immediatly end if collision is detected
					return col; // return true if collision
			}
		}
	}
	// Return false if collision free
	return col;
}

// return true if path has collision
bool check_path_arm(double jnt_ang0[5], double jnt_ang1[5], CollisionObject ybota[5], Transform3f orient_arr[5], CollisionObject obs_arr[12])
{
	double path_length = sqrt(pow((jnt_ang0[0] - jnt_ang1[0]), 2) + pow((jnt_ang0[1] - jnt_ang1[1]), 2) + pow((jnt_ang0[2] - jnt_ang1[2]), 2) + pow((jnt_ang0[3] - jnt_ang1[3]), 2) + pow((jnt_ang0[4] - jnt_ang1[4]), 2));
	double num_steps = round(path_length / 0.05);
	//cout << "pathlength is: " << path_length << ", number of steps is: " << num_steps << endl;
	//cout << num_steps << endl;
	double jnt_ang[5];

	bool col = false; // true for collision
	double lambda;

	CollisionObject *ybota_temp; // temporary storage of youbot bounding boxes, need pointer to return array

	// Check generated sample (if no path)
	if (num_steps == 0) // i.e. if (x0 == x1 && y0 == y1)
	{
		ybota_temp = set_youbot_arm(jnt_ang, ybota, orient_arr); // apply FWD kinematics
		//ybota_temp = set_youbot_arm(jnt_ang0,ybota); // apply FWD kinematics
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
	else // Check entire path
	{
		// Check for collisions all along path
		for (int i = 0; i < num_steps; i++)
		{
			// Interpolate each joint angle at given step
			lambda = (i + 1) / num_steps;
			for (int k = 0; k < 5; k++) { jnt_ang[k] = (1 - lambda)*jnt_ang0[k] + lambda*jnt_ang1[k]; }

			//cout << "jnt_ang[0] is: " << jnt_ang[0] << endl;
			//cout << "jnt_ang[1] is: " << jnt_ang[1] << endl;

			ybota_temp = set_youbot_arm(jnt_ang, ybota, orient_arr); // apply FWD kinematics
			//ybota_temp = set_youbot_arm(jnt_ang,ybota); // apply FWD kinematics
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
	}
	// Return false if collision free
	return col;
}
