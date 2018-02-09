/*
* RRTmanipulator.h
*
*  Created on: May 1, 2016
*      Author: usman
*/

#ifndef RRTMANIPULATOR_H_
#define RRTMANIPULATOR_H_

#include "matrix.h"
#include "manipulatorconfig.h"
#include <math.h>
#include <list>

class RRT_manipulator {
public:
	RRT_manipulator(manipulator_config, int, float);
	bool add_node();
	int add_node(manipulator_config);
	manipulator_config nearest_neighbour(manipulator_config);
	manipulator_config nearest_neighbour(manipulator_config, float);
	float dist_manipulator(manipulator_config, manipulator_config);
	void plan_path();
	list<manipulator_config> get_path(manipulator_config);
	bool edge_check(manipulator_config, manipulator_config);
	bool collision_check(manipulator_config, manipulator_config);
	virtual ~RRT_manipulator();

private:
	Matrix<manipulator_config>  config_mat;		// configurations present in the graph
	Matrix<int> 				adjacency_mat;  // connectivity of graph
	manipulator_config 			q;		   		// used for generating random configurations
	manipulator_config			q_near;			// nearest neighbor configuration
	manipulator_config			q_temp;
	float 						step_size; 		// step along unit vector
	int 						node_count;		// no. of nodes currently in RRT
	int 						max_nodes;		// Max no. of nodes in RRT

	list<manipulator_config>    path;
};

RRT_manipulator::RRT_manipulator(manipulator_config q_init, int total_nodes, float step)
{
	node_count = 0;
	max_nodes = total_nodes;
	step_size = step;

	adjacency_mat.reset_matrix(max_nodes + 1, max_nodes + 1);
	config_mat.reset_matrix(max_nodes + 1, max_nodes + 1);

	adjacency_mat.put(node_count, node_count, 1);
	config_mat.put(node_count, node_count, q_init);
	node_count++;

}

//***************************************************************************************************************

bool RRT_manipulator::add_node()
{
	q.generate_config();

	float th1, th2, th3, th4;

	// Next check collision avoidance for this node
	if (collision_check(q, q) == false)
	{
		q_near = nearest_neighbour(q);

		// Find q_new at a distance of step_size which will be added to RRT
		th1 = q.get_theta(1) - q_near.get_theta(1);
		th2 = q.get_theta(2) - q_near.get_theta(2);
		th3 = q.get_theta(3) - q_near.get_theta(3);
		th4 = q.get_theta(4) - q_near.get_theta(4);

		float vec_mag = dist_manipulator(q, q_near);

		if (vec_mag>step_size)
		{
			th1 = th1 / vec_mag;
			th2 = th2 / vec_mag;
			th3 = th3 / vec_mag;
			th4 = th4 / vec_mag;

			th1 = q_near.get_theta(1) + step_size*th1;
			th2 = q_near.get_theta(2) + step_size*th2;
			th3 = q_near.get_theta(3) + step_size*th3;
			th4 = q_near.get_theta(4) + step_size*th4;

			// q_new is assigned new ID
			q.set(th1, th2, th3, th4, node_count, q_near.get_id());
		}
		else
			q.set(q.get_theta(1), q.get_theta(2), q.get_theta(3), q.get_theta(4), node_count, q_near.get_id());

		if (edge_check(q_near, q) == false)
		{
			// q_new is added to graph
			adjacency_mat.put(node_count, q_near.get_id(), 1);
			config_mat.put(node_count, q_near.get_id(), q);
			node_count++;
		}

	}
	return true;
}

//***************************************************************************************************************

int RRT_manipulator::add_node(manipulator_config target)
{
	bool flag = false;
	q_near = nearest_neighbour(target);
	q.set(target.get_theta(1), target.get_theta(2), target.get_theta(3), target.get_theta(4), max_nodes, q_near.get_id());

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
			q_near = nearest_neighbour(target, dist_manipulator(q_near, target));
	}

	return q_near.get_id();
}



//***************************************************************************************************************

float RRT_manipulator::dist_manipulator(manipulator_config q1, manipulator_config q2)
{

	float Dist = 0;
	Dist = sqrt(pow(q1.get_theta(1) - q2.get_theta(1), 2) + pow(q1.get_theta(2) - q2.get_theta(2), 2) + pow(q1.get_theta(3) - q2.get_theta(3), 2) + pow(q1.get_theta(4) - q2.get_theta(4), 2));

	return Dist;
}

//***************************************************************************************************************


manipulator_config RRT_manipulator::nearest_neighbour(manipulator_config query)
{
	int i, j, xhat, yhat;
	float dist = 10000; // some large distance value
	int id_qnear = -1;

	for (i = 0; i <= node_count; i++)
	{
		for (j = 0; j <= node_count; j++)
		{
			q_temp = config_mat.get(i, j);
			if (q_temp.get_id()>-1)		// A node with positive ID
			{
				if (dist_manipulator(q_temp, query)<dist)
				{
					xhat = i;
					yhat = j;
					id_qnear = q_temp.get_id();
					dist = dist_manipulator(q_temp, query);
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


manipulator_config RRT_manipulator::nearest_neighbour(manipulator_config query, float min_dist)
{
	int i, j, xhat, yhat;
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
				temp_dist = dist_manipulator(q_temp, query);
				if (temp_dist<dist && temp_dist>min_dist)
				{
					xhat = i;
					yhat = j;
					id_qnear = q_temp.get_id();
					dist = dist_manipulator(q_temp, query);
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

void RRT_manipulator::plan_path()
{
	while (node_count<max_nodes)
		add_node();

}

//***************************************************************************************************************

bool RRT_manipulator::edge_check(manipulator_config q1, manipulator_config q2)
{
	float q1th1, q1th2, q1th3, q1th4, q2th1, q2th2, q2th3, q2th4, dth1, dth2, dth3, dth4;

	q1th1 = q1.get_theta(1);
	q1th2 = q1.get_theta(2);
	q1th3 = q1.get_theta(3);
	q1th4 = q1.get_theta(4);

	q2th1 = q2.get_theta(1);
	q2th2 = q2.get_theta(2);
	q2th3 = q2.get_theta(3);
	q2th4 = q2.get_theta(4);

	dth1 = (q2th1 - q1th1) / 10;
	dth2 = (q2th2 - q1th2) / 10;
	dth3 = (q2th3 - q1th3) / 10;
	dth4 = (q2th4 - q1th4) / 10;

	bool flag = false;
	int ind = 0;

	/*
	while(flag==false, ind<10)
	{
	q.set(q1.get_theta(1)+dth1*ind,q1.get_theta(2)+dth2*ind,q1.get_theta(3)+dth3*ind,q1.get_theta(4)+dth4*ind,q1.get_id(),q1.get_parent_id());
	flag=collision_check(q);
	ind++;
	}
	*/
	flag = collision_check(q1, q2);
	return flag;
}



//***************************************************************************************************************

bool RRT_manipulator::collision_check(manipulator_config q1, manipulator_config q2)
{

	// q0 is start, q1 is goal
	CollisionObject *obs_arr;
	CollisionObject obs_arr_temp[12];
	obs_arr = set_obstacles(obs_arr_temp);
	CollisionObject ybota[5];
	Transform3f orient_arr[5];

	double jnt_ang0[5];
	double jnt_ang1[5];
	for (int i = 0; i<4; i++)
	{
		jnt_ang0[i] = q1.get_theta(i + 1);
		jnt_ang1[i] = q2.get_theta(i + 1);
	}
	jnt_ang0[4] = 0; // dummy value
	jnt_ang1[4] = 0; // dummy value

	bool col = check_path_arm(jnt_ang0, jnt_ang1, ybota, orient_arr, obs_arr);
	//bool col = false;
	return col;

}

//***************************************************************************************************************

list<manipulator_config> RRT_manipulator::get_path(manipulator_config q_goal)
{
	Matrix<int> 		Amatrix;
	Matrix<manipulator_config> Cmatrix;
	int parent_row;

	// First add target node
	parent_row = add_node(q_goal);

	q = q_goal;
	q.set(q.get_theta(1), q.get_theta(2), q.get_theta(3), q.get_theta(4), max_nodes, parent_row);

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
		//q.print_config();
		path.push_front(q);
	}
	return path;
}

//***************************************************************************************************************

RRT_manipulator::~RRT_manipulator() {
	// TODO Auto-generated destructor stub
}



#endif /* RRTMANIPULATOR_H_ */
