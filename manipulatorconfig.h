/*
* manipulatorconfig.h
*
*  Created on: May 1, 2016
*      Author: usman
*/

#ifndef MANIPULATORCONFIG_H_
#define MANIPULATORCONFIG_H_

using namespace std;
#include <iostream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "structures.h"

#define PI 3.1416

class manipulator_config {
public:
	manipulator_config();
	manipulator_config(int);
	manipulator_config(float, float, float, float, int);
	Configuration generate_config(int = 0);
	void   print_config();
	float get_theta(int);
	int	  get_id();
	int	  get_parent_id();
	void set(float, float, float, float, int, int);
	bool operator==(manipulator_config);
	virtual ~manipulator_config();

private:
	Configuration config;
	static const float theta1_low_limit = -2.94;
	static const float theta1_upp_limit = 2.94;
	static const float theta2_low_limit = -1.13;
	static const float theta2_upp_limit = 1.57;
	static const float theta3_low_limit = -2.63;
	static const float theta3_upp_limit = 2.54;
	static const float theta4_low_limit = -1.78;
	static const float theta4_upp_limit = 1.78;

};

manipulator_config::manipulator_config()
{
	config.Id = -1;

	srand(time(NULL));
}

manipulator_config::manipulator_config(int)
{
	config.Id = -1;
	config.theta1 = theta1_low_limit;
	config.theta2 = theta2_low_limit;
	config.theta3 = theta3_low_limit;
	config.theta4 = theta4_low_limit;

	srand(time(NULL));
}

manipulator_config::manipulator_config(float th1, float th2, float th3, float th4, int ID)
{
	config.Id = ID;
	config.theta1 = th1;
	config.theta2 = th2;
	config.theta3 = th3;
	config.theta4 = th4;

	srand(time(NULL));
}

Configuration manipulator_config::generate_config(int node_id)
{
	config.theta1 = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*(theta1_upp_limit - theta1_low_limit);
	config.theta2 = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*(theta2_upp_limit - theta2_low_limit);
	config.theta3 = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*(theta4_upp_limit - theta3_low_limit);
	config.theta4 = (static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*(theta4_upp_limit - theta3_low_limit);
	config.Id = node_id - 1;
	return config;
}

void manipulator_config::print_config()
{
	cout << "Random sample generated at (" << config.theta1 << "," << config.theta2 << "," << config.theta3 << "," << config.theta4 << "," << config.Id << ")\n";
}


float manipulator_config::get_theta(int ind)
{
	switch (ind)
	{
	case 1:
		return config.theta1;
	case 2:
		return config.theta2;
	case 3:
		return config.theta3;
	case 4:
		return config.theta4;
	}
}

int	  manipulator_config::get_id()
{
	return config.Id;
}

int	  manipulator_config::get_parent_id()
{
	return config.parent_Id;
}

void manipulator_config::set(float th1, float th2, float th3, float th4, int id, int p_id)
{
	config.Id = id;
	config.parent_Id = p_id;
	config.theta1 = th1;
	config.theta2 = th2;
	config.theta3 = th3;
	config.theta4 = th4;
}

bool manipulator_config::operator==(manipulator_config test)
{
	float th1 = test.get_theta(1);
	float th2 = test.get_theta(2);
	float th3 = test.get_theta(3);
	float th4 = test.get_theta(4);
	int id = test.get_id();
	if (th1 == config.theta1 && th2 == config.theta2 && th3 == config.theta3 && th4 == config.theta4 && id == config.Id)
		return true;
	else
		return false;
}


manipulator_config::~manipulator_config() {
	// TODO Auto-generated destructor stub
}



#endif /* MANIPULATORCONFIG_H_ */
