#ifndef DCAS_H
#define DCAS_H

#include <string>
#include <mutex>
#include <map>
#include <memory>
#include "drone.h"


class dcas
{
private:
public:
	//The size of one row of the map.  The entire map consists of mapSize*mapSize spaces.
	int mapSize;
	
	//Used to prevent situations where returning drones are attempting to reach the airport but 
	//a drone has just taken off.
	bool blockTakeoff;
	
	//Contains a mutex for each space on the map
	std::map<int, std::unique_ptr<std::mutex>> locks;
	
	//Used to display a representation of airport and drone locations to users.
	//Contains one of 3 possible symbols.  'B' denoting the location of an airport, 'D' 
	//denoting the location of a drone, and ' ' denoting an empty space.
	std::map<int, char> map;

	dcas();
	
	//Method ran by threads to create a drone and call the navigate function on them
    static void *controlDrone(void* arguments);
	
	//Handles all navigation from takeoff to landing
	void Navigate(Drone drone);
	
	//Ran by constructor to generate drones and mapSize from input.txt.  Then creates
	//a thread for each drone represented by the input and passes them to controlDrone.
	void makeDrones();
	
	//Prints the current state of the map, providing a visualization of where drones are
	//currently located
    void printMap();
	
	//Used by a drone to request takeoff if in state 0 or request landing if in state 2
	//and currently on map location 1
	void requestTakeoffLand(Drone &drone);
	
	//Converts integer representation of locations on the map to x,y coordinates for use
	//in the navigate function
	std::pair<int, int> intToXY(int coord);

	//Attempts to move drone to moveLoc by calling try_lock on the relevant mutex.  Returns
	//true and updates the map if try_lock succeeds and returns false otherwise.
	bool tryMove(int moveLoc, Drone &drone);
	
	//Used by navigate to prevent deadlock by forcing a drone that has not moved previously
	//to attempt to move in any direction following a set order
	bool preventDeadlock(Drone &drone, int x, int y);
};

//Used to pass multiple arguments to the controlDrone method
struct argStruct{
	dcas* base;
	int goal;
};
#endif	// DCAS_H