#include "dcas.h"
#include "drone.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <memory>
#include <map>
#include <vector>
#include <list>
#include <unistd.h>

//Prints the current state of the map
void dcas::printMap()
{
	std::string line = "";
	for (int i = 1; i <= this->mapSize*this->mapSize; i++){
		line = line + '[' + this->map[i] + ']';
		if (i % this->mapSize == 0){
			line += '\n';
		}
	}
	line += '\n';
	std::cout << line;
}

//Converts an integer to the x,y coordinate that the interger represents
std::pair<int, int> dcas::intToXY(int coord){
	int x;
	int y;
	if (coord % this->mapSize == 0){
		x = this->mapSize;
		y = coord / x;
		return std::pair<int, int> (x, y);
	}
	else{
		x = coord % this->mapSize;
		y = coord / x;
		return std::pair<int, int> (x, y);
	}
}

//Attempts to require a lock for the desired position, upon success, updates
//the drone's location to the desired one, updates the map, and releases the 
//lock for the space it previously occupied.  Upon failure, returns false.
bool dcas::tryMove(int moveLoc, Drone &drone){
	if (this->locks[moveLoc]->try_lock()){
		int release = drone.location;
		drone.location = moveLoc;
		if (release != 1){
			this->map[release] = ' ';
		}
		else{
			this->map[release] = 'B';
		}
		this->map[moveLoc] = 'D';
		this->printMap();
		this->locks[release]->unlock();
		return true;
	}
	else{
		return false;
	}
}

//Used by drones to begin and end their travel based on their current state
void dcas::requestTakeoffLand(Drone &drone){
	//If try_lock fails there are 2 possibilities, either this drone is attempting
	//takeoff while another drone is occupying the space or the drone is attempting
	//to land and thus already holds the lock for the space
	if (this->locks[1]->try_lock()){
		drone.state = 1;
		this->map[1] = 'D';
		this->printMap();
	}
	//If the drone is in state 2, it already possesses the lock and is clear to land
	//which will transition it to state 3 (landed) and release the lock for the space
	else if (drone.state == 2) {
		this->map[1] = 'B';
		this->locks[1]->unlock();
		this->printMap();
		if (this->map[2] == ' ' && this->map[1 + mapSize] == ' '){
			this->blockTakeoff = false;
		}
		drone.state = 3;
	}
}

//Tries to prevent deadlock by having a drone move in any direction if it could not move in a relevant direction
//drones will always attempt to move in the following order: x+1,y+1,x-1,y-1.  If one of these moves succeed,
//true is returned.  If none of the moves succeed, false is returned.
bool dcas::preventDeadlock(Drone &drone, int x, int y){
	//Each statement first checks to see if the move would leave the bounds of the map.  If the move is ok,
	//it will attempt to move to that location.  Upon successfully moving to the location, true is returned.
	//Avoids allowing drones to move into corners as these are likely to cause deadlock
	if (x < this->mapSize){
		if (drone.location + 1 == this->mapSize || drone.location + 1 == this->mapSize*this->mapSize){
			//do nothing, avoids top and bottom right corners
		}
		else if(tryMove(drone.location + 1, drone)){
			return true;
		}
	}
	if (y < this->mapSize){
		if (drone.location + this->mapSize == this->mapSize*this->mapSize || drone.location + this->mapSize == this->mapSize*this->mapSize-this->mapSize+1){
			//do nothing, avoids bottom corners
		}
		else if(tryMove(drone.location + this->mapSize, drone)){
			return true;
		}
	}
	if (x > 1){
		if (drone.location - 1 == 1 || drone.location - 1 == this->mapSize*this->mapSize-this->mapSize+1){
			//Avoids corners except for the case where a drone has not yet reached its goal and is one move
			//away from the base.  It's likely that this drone has very recently launched and is causing deadlock
			//as it tries to move away from base and other drones are attemping to return.  In this case, we will allow
			//the drone to try to move onto the base so next turn can trigger the drone to land temporarily.
			if (drone.goal != 1 && drone.location - 1 == 1){
				if (tryMove(1, drone)){
					return true;
				}
			}
		}
		else if (tryMove(drone.location - 1, drone)){
			return true;
		}
	}
	if (y > 1){
		//does not allow drones to move back over base, as it's likely
		//to cause deadlock
		if (drone.location - this->mapSize == 1 || drone.location - this->mapSize == this->mapSize){
			//Avoids corners except for the case where a drone has not yet reached its goal and is one move
			//away from the base.  It's likely that this drone has very recently launched and is causing deadlock
			//as it tries to move away from base and other drones are attemping to return.  In this case, we will allow
			//the drone to try to move onto the base so next turn can trigger the drone to land temporarily.
			if (drone.goal != 1 && drone.location - this->mapSize == 1){
				if (tryMove(drone.location - this->mapSize, drone)){
					return true;
				}
			}
		}
		else if (tryMove(drone.location - this->mapSize, drone)){
			return true;
		}
	}
	if (drone.location == 1 && drone.state == 1){
		//We've just taken off, but other drones are trying to land and are being blocked.  Land, revert
		//state to 1, and attempt to takeoff again after giving others the chance to occupy the base for
		//landing
		drone.state = 0;
		this->map[1] = 'B';
		this->locks[1]->unlock();
		this->printMap();
	}
	//if we get here, all move attempts have failed and movedLast remains false
	return false;
}

//Controls all navigation of drones based on their current state and goal.  Exits
//upon landing when a drone reaches state 3.
void dcas::Navigate(Drone drone){
	//Used to assist in deadlock detection.  Denotes if a drone was able to move
	//this turn.
	bool movedLast;
	//When a drone has reached state 3 (landed), navigation is complete
	while (drone.state != 3){
		switch(drone.state){
			case 0:
				//Attempts to acquire the lock for the space where the base is located
				//Will fail if another drone is already occupying the space.
				while (blockTakeoff){
					//Spin until blockTakeoff = false, prevents drones from taking off
					//well the spaces 2 and 1 + mapSize are occupied
				}
				requestTakeoffLand(drone);
				break;
			case 1:
				if (drone.location == drone.goal){
					//If the drone is at its goal location, and that goal is 1, the drone
					//has already reached its original goal and has now returned to base
					//it will now attempt to land at the airport
					if (drone.goal == 1){
						drone.state = 2;
					}
					//Otherwise, the drone has just reached its goal and now needs to return
					//to the airport, located at space 1.
					else{
						drone.goal = 1;	
					}
				}
				else{
					//convert numerical location to coordinates
					std::pair<int, int> conversion = intToXY(drone.location);
					int x = conversion.first;
					int y = conversion.second;
					
					//convert numerical goal to coordinates
					std::pair<int, int> goalConversion = intToXY(drone.goal);
					int goalx = goalConversion.first;
					int goaly = goalConversion.second;
					
					//always attempts to reach it's goalx before attempting to reach goaly
					if (x < goalx){
						if (tryMove(drone.location + 1, drone)){
							movedLast = true;
						}
						//if unable to move closer to goalx, the drone will attempt to move closer to goaly if not already there
						else if (y != goaly){
							if (y < goaly){
								if (tryMove(drone.location + this->mapSize, drone)){
									movedLast = true;
								}
								else{movedLast = false;}
							}
							else {
								if (tryMove(drone.location - this->mapSize, drone)){
									movedLast = true;
								}
								else{movedLast = false;}
							}
						}
						else {movedLast = false;}
					}
					else if (x > goalx){
						if (tryMove(drone.location - 1, drone)){
							movedLast = true;
						}
						else if (y != goaly){
							if (y < goaly){
								if (tryMove(drone.location + this->mapSize, drone)){
									movedLast = true;
								}
							}
							else {
								if (tryMove(drone.location - this->mapSize, drone)){
									movedLast = true;
								}
							}
						}
						else {movedLast = false;}
					}
					//goalx is already reached, attempt to reach goaly
					else {
						if (y < goaly){
							if (tryMove(drone.location + this->mapSize, drone)){
								movedLast = true;
							}
						}
						else {
							if (tryMove(drone.location - this->mapSize, drone)){
								movedLast = true;
							}
							else{movedLast = false;}
						}
					}
					if (!movedLast){
						//redirects to method that attempts to prevent deadlock, see above
						movedLast = dcas::preventDeadlock(drone, x, y);
					}
					if (drone.location == 2 || drone.location == 1 + this->mapSize){
						this->blockTakeoff = true;
					}
				}
				break;
			case 2:
				//Drone has completed its delivery and is now hovering over the airport
				//attempting to land
				dcas::requestTakeoffLand(drone);
				break;
		}
	}
}

//Creates a drone from the arguments then calls Navigate to begin the drone's travel
void *dcas::controlDrone(void* arguments){
	argStruct *args = (argStruct *) arguments;
	Drone drone = Drone(args->goal);
	args->base->Navigate(drone);
	return nullptr;
}

//Reads input.txt to establish the mapSize, number of drones, and drone targets
void dcas::makeDrones(){
	int i;
	int goal;
	int threadCount = 0;
	bool firstLine = true;
    std::ifstream inFile;
    std::string line;
	std::vector<pthread_t> threads;
 
    inFile.open("input.txt");
    if (!inFile) {
            std::cerr << "Can't open input.txt";
            exit(1);
    }
	pthread_t thread[20];
	dcas* arg;
	arg = this;
    while (std::getline(inFile, line)){
		//Determines the size of the map then builds the char and mutex maps accordingly
		if (firstLine){
            this->mapSize = std::stoi(line);
            this->map.insert(std::make_pair(1, 'B'));
			this->locks.insert(std::make_pair(1, std::make_unique<std::mutex>()));
			for (i = 2; i <= this->mapSize*this->mapSize; i++){
                this->map.insert(std::make_pair(i, ' '));
				this->locks.insert(std::make_pair(i, std::make_unique<std::mutex>()));
            }
            firstLine = false;
			this->printMap();
        }
		//Each line that follows denoted a new drone with a goal located at the text of the line
        else{
            //pthread_t thread;
			goal = std::stoi(line);
			argStruct* argPointer;
			argPointer = new argStruct;
			argPointer->base = this;
			argPointer->goal = goal;
            pthread_create(&thread[threadCount], NULL, dcas::controlDrone,(void *) argPointer);
			threadCount++;
        }
    }
	for (i = 0; i < threadCount; i++){
		pthread_join(thread[i], NULL);
	}
    //inFile.close();
}    

dcas::dcas(){
	this->blockTakeoff = false;
	makeDrones();
}

int main()
{
	dcas base = dcas();
    return 0;
	pthread_exit(0);
}
