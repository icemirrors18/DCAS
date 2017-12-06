#include "drone.h"
#include <map>


Drone::Drone(int goals){
	state = 0;
	location = 1;
	goal = goals;
}