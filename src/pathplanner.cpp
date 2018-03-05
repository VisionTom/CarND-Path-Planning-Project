#include "pathplanner.h"
#include <iostream>
#include <math.h>
#include "spline.h"

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }


PathPlanner::PathPlanner()
{
}

vector<double> PathPlanner::get_next_x_vals(){
	return next_x_vals;
}

vector<double> PathPlanner::get_next_y_vals(){
	return next_y_vals;
}

void PathPlanner::setParameters(vector<double>& x,vector<double>& y,vector<double>& s,vector<double>& dx,vector<double>& dy){
	map_waypoints_x = x;
	map_waypoints_y = y;
	map_waypoints_s = s;
	map_waypoints_dx = dx;
	map_waypoints_dy = dy;
}

void PathPlanner::extractParametersFromJson(json j){
	//reset previous Path
	next_x_vals.clear();
	next_y_vals.clear();

	// Main car's localization Data
	car_x = j[1]["x"];
	car_y = j[1]["y"];
	car_s = j[1]["s"];
	car_d = j[1]["d"];
	car_yaw = j[1]["yaw"];
	car_speed = j[1]["speed"];

	// Previous path data given to the Planner
	vector<double> x = j[1]["previous_path_x"];
	vector<double> y = j[1]["previous_path_y"];

	previous_path_x.swap(x);
	previous_path_y.swap(y);

	// Previous path's end s and d values 
	end_path_s = j[1]["end_path_s"];
	end_path_d = j[1]["end_path_d"];

	//ToDo
	// Sensor Fusion Data, a list of all other cars on the same side of the road.
	vector<vector<double>> sf = j[1]["sensor_fusion"];
	sensorfusion.swap(sf);
}

//Test#1: Simply drive in a straight line 
void PathPlanner::driveStraightLine(){

	for(int i = 0; i < FUTURE_PATH_SIZE; i++)
	{
		next_x_vals.push_back(car_x+(DIST_INC*i)*cos(deg2rad(car_yaw)));
		next_y_vals.push_back(car_y+(DIST_INC*i)*sin(deg2rad(car_yaw)));
	}

}

//Test#2: Drive the car in Circles
void PathPlanner::driveCircles(){
	double pos_x;
	double pos_y;
	double angle;
	int path_size = previous_path_x.size();

	for(int i = 0; i < path_size; i++)
	{
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	if(path_size == 0)
	{
		pos_x = car_x;
		pos_y = car_y;
		angle = deg2rad(car_yaw);
	}
	else
	{
		pos_x = previous_path_x[path_size-1];
		pos_y = previous_path_y[path_size-1];

		double pos_x2 = previous_path_x[path_size-2];
		double pos_y2 = previous_path_y[path_size-2];
		angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
	}


	for(int i = 0; i < FUTURE_PATH_SIZE-path_size; i++)
	{    
		next_x_vals.push_back(pos_x+(DIST_INC)*cos(angle+(i+1)*(M_PI/100)));
		next_y_vals.push_back(pos_y+(DIST_INC)*sin(angle+(i+1)*(M_PI/100)));
		pos_x += (DIST_INC)*cos(angle+(i+1)*(M_PI/100));
		pos_y += (DIST_INC)*sin(angle+(i+1)*(M_PI/100));
	}
}

//Test #3: Follow the current line. Problem: Harsh turns cause high jerks
void PathPlanner::followLane(){

	for(int i = 0; i < FUTURE_PATH_SIZE; i++)
	{
		double next_s = car_s+(i+1)*DIST_INC;
		double next_d = 6;
		vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		next_x_vals.push_back(xy[0]);
		next_y_vals.push_back(xy[1]);
	}
}

void PathPlanner::global2local(){
}

void PathPlanner::local2global(){
	
}

//Follow the current line using Splines
void PathPlanner::followLane_with_Splines(){

	check_CarInFront();

	int prev_size = previous_path_x.size();

	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

    // calculate current and last state
	if (prev_size < 2){
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(car_x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(car_y);
	}
	else{
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

    //In Frenet add spaced points ahead of the starting reference
	vector<double> next_wp1 = getXY(car_s+55, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	//Global coordinate to Local coordinate
	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i]-ref_x;
		double shift_y = ptsy[i]-ref_y;

		ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
		ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	}

    // create a spline
	tk::spline s;

    //set (x,y) points to the spline
    s.set_points(ptsx, ptsy); // the anchor points/waypoints

    next_x_vals.clear();
    next_y_vals.clear();

    // generate next path planning points, based on waypoints
    for (int i = 0; i < previous_path_x.size(); i++)
    {
    	next_x_vals.push_back(previous_path_x[i]);
    	next_y_vals.push_back(previous_path_y[i]);
    }

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

    double x_add_on = 0; //since we are starting at the origin

    // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for (int i = 1; i <= 50-previous_path_x.size(); i++)
    {
      double N = (target_dist/(0.02*ref_vel/2.24)); // 2.24: conversion to m/s
      double x_point = x_add_on+(target_x)/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      // go back to global coordinate
      x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
}

void PathPlanner::check_lane_change(){

	//lane_change_delay is a counter, so the car does not change lanes too fast again.
	if(lane_change_delay>0){
		cout << "Lanechange_Delay: " << lane_change_delay << endl;
		return;
	}

	//Init
	vector<int> lanes_possible;
	double distance_to_closest_car_in_lane_in_front[3]; //Saving the distances, for when we have more than 1 possible lane to change
		
	distance_to_closest_car_in_lane_in_front[0] = 99999;
	distance_to_closest_car_in_lane_in_front[1] = 99999;
	distance_to_closest_car_in_lane_in_front[2] = 99999;

	if(lane==1){
		lanes_possible.push_back(0); 
		lanes_possible.push_back(2);
		cout << "I have to break - can i change to lane 0 or 2?" << endl;
	}
	else{
		lanes_possible.push_back(1);
		cout << "I have to break - can i change to lane 1?" << endl;
	}

	//Check for all possible lanes, if a change is possible
	for(int l=0;l<lanes_possible.size();l++){
		
		//Check all cars from sensorfusion
		for (int i=0;i<sensorfusion.size();i++){
			float otherCar_d = sensorfusion[i][6]; //#get the d-value (horizontal position) from the i-th car

			//Is the car is in the lane we want to change into?
			if((otherCar_d < (4*(lanes_possible[l]+1)) && otherCar_d > (4*lanes_possible[l]))){
				
				//Distance between my car and the car #i
				double otherCar_s = sensorfusion[i][5]; //#get the s-value from the i-th car
				double distance_to_car = otherCar_s-car_s;

				//lane in lanes_possible[l] is checked for cars in front. 
				//The array distance_to_closest_car_in_lane_in_front will have the closest car in front for every lane.
				//This is used if several lanes are possible
				if((distance_to_car > 0) && (distance_to_car < distance_to_closest_car_in_lane_in_front[lanes_possible[l]])){
					distance_to_closest_car_in_lane_in_front[lanes_possible[l]] = distance_to_car;
					cout << "There is a car on lane " << lanes_possible[l] << " with a distance of" << distance_to_car << endl;
				}

				//Check if there is a car in a lane to change.
				//Check if the car is inside the SAFETY-zone (from +SAFETY_DISTANCE to SAFETY_DISTANCE/3) 
				if((distance_to_car<SAFETY_DISTANCE) && distance_to_car>(SAFETY_DISTANCE/-3)){	
					cout << "I can't change to lane " << lanes_possible[l] << ". The distance is only " << distance_to_car << endl;
					lanes_possible.erase(lanes_possible.begin()+l); //Car is in the way. Lanechange to index l is not possible.
					l--; //because of the erase, to get the order right
					
					break;
				}
			}
		}
	}
	
	//Going through the checks and decide for the best lane to change.
	for(int l=0;l<lanes_possible.size();l++){
		if(lanes_possible.size()>1){
			if(distance_to_closest_car_in_lane_in_front[lanes_possible[0]] > distance_to_closest_car_in_lane_in_front[lanes_possible[1]]){
				lane = lanes_possible[0];
				cout << "** Choosing between 0 and 2... Let's have a look on the cars in front of these lanes: " << endl;
				cout << "The car in front in lane " << lanes_possible[0] << " has a diststance of: "<< distance_to_closest_car_in_lane_in_front[lanes_possible[0]] << endl;
				cout << "The car in front in lane " << lanes_possible[1] << " has a diststance of: "<< distance_to_closest_car_in_lane_in_front[lanes_possible[1]] << endl;
				cout << "Well, lane " << lanes_possible[0]  << " has more free space. Let's go there!" << endl << endl;	
			}
			else{
				lane = lanes_possible[1];

				cout << "** Choosing between 0 and 2... Let's have a look on the cars in front of these lanes: " << endl;
				cout << "The car in front in lane " << lanes_possible[0] << " has a diststance of: " << distance_to_closest_car_in_lane_in_front[lanes_possible[0]] << endl;
				cout << "The car in front in lane " << lanes_possible[1] << " has a diststance of: " << distance_to_closest_car_in_lane_in_front[lanes_possible[1]] << endl;
				cout << "Well, lane " << lanes_possible[1]  << " has more free space. Let's go there!" << endl << endl;	
			}
		}
		else{
			lane = lanes_possible[l];	//Change lane
			cout << "Lane " << lanes_possible[l] << " is good. Choosing this lane. BYEBYE." << endl;
		}
		lane_change_delay = 50;
	}
}

//Test 5: Check if there is a car in front and prevent collision
void PathPlanner::check_CarInFront(){

	bool too_close, very_close, emergency_close = false; 
	double otherCar_speed = -1;

	//Checking for distances to other cars and their speed.
	for (int i=0;i<sensorfusion.size();i++){
		float otherCar_d = sensorfusion[i][6]; //#get the d-value (horizontal position) from the i-th car
		
		//If the car is in our lane
		if(otherCar_d < (4*(lane+1)) && otherCar_d > (4*lane)){
			double otherCar_s = sensorfusion[i][5]; //#get the s-value from the i-th car
			double distance_to_car = otherCar_s-car_s;
			
			if(distance_to_car<SAFETY_DISTANCE && distance_to_car>0){
				too_close = true;
				double otherCar_vx = sensorfusion[i][3];
				double otherCar_vy = sensorfusion[i][4];
				otherCar_speed = sqrt(abs((otherCar_vx)*(otherCar_vx))+abs((otherCar_vy)*(otherCar_vy))); //pythagoras 
				otherCar_speed = otherCar_speed * 3.6 / 1.6; // convertion from m/s to kmh. and then from km/h to mp/h

				if(distance_to_car<SAFETY_DISTANCE/3){
					very_close = true;
					if(distance_to_car<SAFETY_DISTANCE/6){
						emergency_close = true;
					}
				}
			}
		}
	}

	//Regulating the speed and distance 
	if(too_close){
		cout << "car_speed: " << car_speed << "     otherCar_speed: " << otherCar_speed << endl;
		check_lane_change();

		//don't drive a lower speed than the car in front of our car
		if(!(car_speed < otherCar_speed) && car_speed > 1){
			ref_vel-=0.3;

			if(very_close){
				cout << "Harsher break." << endl;
				ref_vel-=0.8;

				if(emergency_close){
					cout << "EMERGENCY BREAK. HOLD YOUR HAT." << endl;
					ref_vel-=3;
				}
			}
		}
	}

	//Max Speed
	else if(ref_vel<MAX_SPEED){
		ref_vel+=0.8;
	}
	//don't drive backwards
	if(ref_vel<=0){
		ref_vel = 0;
	}

	lane_change_delay--;
}

