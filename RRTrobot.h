/*
* RRTrobot.h
*
*  Created on: May 1, 2016
*      Author: usman
*/

#ifndef RRTROBOT_H_
#define RRTROBOT_H_

#include "robotconfig.h"
#include "collisionchecker.h"
#include "matrix.h"
#include <math.h>
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <list>

//using namespace cv;

class RRT_robot {
public:
	RRT_robot(robot_config, int, float);
	bool add_node();
	int add_node(robot_config);
	robot_config nearest_neighbour(robot_config);
	robot_config nearest_neighbour(robot_config, float);
	float dist_robot_base(robot_config, robot_config);
	void plan_path();
	list<robot_config> get_path(robot_config);
	bool edge_check(robot_config, robot_config);
	bool collision_check(robot_config, robot_config);
	virtual ~RRT_robot();
private:
	Matrix<robot_config> config_mat;		// configurations present in the graph
	Matrix<int> 		adjacency_mat;  // connectivity of graph
	robot_config 		q;		   		// used for generating random configurations
	robot_config		q_near;			// nearest neighbor configuration
	robot_config		q_temp;
	float 				step_size; 		// step along unit vector
	int 				node_count;		// no. of nodes currently in RRT
	int 				max_nodes;		// Max no. of nodes in RRT
	float				xhat;      		// x-component of unit vector
	float				yhat;      		// y-component of unit vector

	//	Mat 				image;
	list<robot_config>   path;
};


RRT_robot::RRT_robot(robot_config q_start, int total_nodes, float step)
{
	//	image.create(500,500,CV_8UC3);
	//	image.setTo(Scalar(255,255,255));

	node_count = 0;
	max_nodes = total_nodes;
	step_size = step;
	xhat = 0;
	yhat = 0;

	adjacency_mat.reset_matrix(max_nodes + 1, max_nodes + 1);
	config_mat.reset_matrix(max_nodes + 1, max_nodes + 1);

	adjacency_mat.put(node_count, node_count, 1);
	config_mat.put(node_count, node_count, q_start);
	node_count++;
	//	circle( image, Point(q_start.get_x(),q_start.get_y()),5.0,Scalar( 0, 255, 0 ),-1,8.0);
}



//***************************************************************************************************************


bool RRT_robot::add_node()
{
	q.generate_config();

	// Next check collision avoidance for this node
	if (collision_check(q, q) == false)
	{
		q_near = nearest_neighbour(q);

		// Find q_new at a distance of step_size which will be added to RRT
		xhat = q.get_x() - q_near.get_x();
		yhat = q.get_y() - q_near.get_y();

		float vec_mag = sqrt(pow(xhat, 2) + pow(yhat, 2));

		if (vec_mag>step_size)
		{
			xhat = xhat / vec_mag;
			yhat = yhat / vec_mag;

			xhat = q_near.get_x() + step_size*xhat;
			yhat = q_near.get_y() + step_size*yhat;

			// q_new is assigned new ID
			q.set(xhat, yhat, node_count, q_near.get_id());
		}
		else
			q.set(q.get_x(), q.get_y(), node_count, q_near.get_id());

		if (edge_check(q_near, q) == false)
		{
			// q_new is added to graph
			adjacency_mat.put(node_count, q_near.get_id(), 1);
			config_mat.put(node_count, q_near.get_id(), q);

			node_count++;
			//		circle( image, Point(q.get_x(),q.get_y()),3.0,Scalar( 0, 0, 255 ),-1,8.0);
			//		arrowedLine(image,Point(q_near.get_x(),q_near.get_y()),Point(q.get_x(),q.get_y()),Scalar( 0,0,0));

		}
	}
	return true;
}

//***************************************************************************************************************

int RRT_robot::add_node(robot_config target)
{
	bool flag = false;

	q_near = nearest_neighbour(target);
	q.set(target.get_x(), target.get_y(), max_nodes, q_near.get_id());

	while (flag == false)
	{
		if (edge_check(q_near, q) == false)
		{
			// q_target is added to graph
			adjacency_mat.put(node_count, q_near.get_id(), 1);
			config_mat.put(node_count, q_near.get_id(), q);
			node_count++;
			flag = true;
		}
		else
		{
			xhat = sqrt(pow(q_near.get_x() - target.get_x(), 2) + pow(q_near.get_y() - target.get_y(), 2));
			q_near = nearest_neighbour(target, xhat);
		}
	}


	//	circle( image, Point(q.get_x(),q.get_y()),5.0,Scalar( 255, 0, 255 ),-1,8.0);
	//	arrowedLine(image,Point(q_near.get_x(),q_near.get_y()),Point(q.get_x(),q.get_y()),Scalar( 0,0,0));

	return q_near.get_id();
}

//***************************************************************************************************************

robot_config RRT_robot::nearest_neighbour(robot_config query)
{
	int i, j;
	float dist = 10000; // some large distance value
	int id_qnear = -1;

	for (i = 0; i <= node_count; i++)
	{
		for (j = 0; j <= node_count; j++)
		{
			q_temp = config_mat.get(i, j);
			if (q_temp.get_id()>-1)		// A node with positive ID
			{
				if (dist_robot_base(q_temp, query)<dist)
				{
					xhat = i;
					yhat = j;
					id_qnear = q_temp.get_id();
					dist = dist_robot_base(q_temp, query);
				}
			}
		}
	}
	if (id_qnear>-1)
	{
		q_near = config_mat.get(xhat, yhat);
	}

	return q_near;
}

//***************************************************************************************************************

robot_config RRT_robot::nearest_neighbour(robot_config query, float min_dist)
{
	int i, j;
	float dist = 100000; // some large distance value
	int id_qnear = -1;
	float temp_dist;

	for (i = 0; i <= node_count; i++)
	{
		for (j = 0; j <= node_count; j++)
		{
			q_temp = config_mat.get(i, j);
			if (q_temp.get_id()>-1)		// A node with positive ID
			{
				temp_dist = dist_robot_base(q_temp, query);
				if (temp_dist<dist && temp_dist> min_dist)
				{
					xhat = i;
					yhat = j;
					id_qnear = q_temp.get_id();
					dist = dist_robot_base(q_temp, query);
				}
			}
		}
	}
	if (id_qnear>-1)
	{
		q_near = config_mat.get(xhat, yhat);
	}

	return q_near;
}

//***************************************************************************************************************


float RRT_robot::dist_robot_base(robot_config q1, robot_config q2)
{
	return sqrt(pow(q1.get_x() - q2.get_x(), 2) + pow(q1.get_y() - q2.get_y(), 2));
}

//***************************************************************************************************************

void RRT_robot::plan_path()
{
	while (node_count<max_nodes)
		add_node();

	//	return image;

}

//***************************************************************************************************************


bool RRT_robot::edge_check(robot_config q1, robot_config q2)
{
	float x1, x2, y1, y2, dx, dy;
	x1 = q1.get_x();
	x2 = q2.get_x();
	y1 = q1.get_y();
	y2 = q2.get_y();

	dx = (x2 - x1) / 10;
	dy = (y2 - y1) / 10;

	bool flag = false;
	int ind = 0;

	/*
	while(flag==false, ind<10)
	{
	q.set(q1.get_x()+dx*ind,q1.get_y()+dy*ind,q1.get_id(),q1.get_parent_id());
	flag=collision_check(q);
	ind++;
	}
	*/
	flag = collision_check(q1, q2);

	return flag;
}

//***************************************************************************************************************

bool RRT_robot::collision_check(robot_config q1, robot_config q2)
{
	// q0 is start, q1 is goal
	CollisionObject *obs_arr;
	CollisionObject obs_arr_temp[12];
	obs_arr = set_obstacles(obs_arr_temp);

	bool col = check_path_base(q1.get_x(), q1.get_y(), q2.get_x(), q2.get_y(), obs_arr);

	return col;
}

//***************************************************************************************************************

list<robot_config> RRT_robot::get_path(robot_config q_goal)
{
	Matrix<int> 		Amatrix;
	Matrix<robot_config> Cmatrix;
	int parent_row;

	// First add target node
	parent_row = add_node(q_goal);

	q = q_goal;
	q.set(q.get_x(), q.get_y(), max_nodes, parent_row);

	while (q.get_id() != 0)
	{
		parent_row = q.get_parent_id();
		for (int col = 0; col<max_nodes; col++)
		{
			if (adjacency_mat.get(parent_row, col) == 1)
			{
				q = config_mat.get(parent_row, col);
				col = max_nodes;
			}
		}
		path.push_front(q);

	}

	return path;
}



RRT_robot::~RRT_robot() {
	// TODO Auto-generated destructor stub
}


#endif /* RRTROBOT_H_ */
