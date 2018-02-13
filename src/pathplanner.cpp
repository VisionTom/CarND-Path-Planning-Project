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
	//auto sensor_fusion = j[1]["sensor_fusion"];

	//cout << "car_x: " << car_x << "\ncar_y: " << car_y << endl;
}

//Test#1: Simply drive in a straight line 
void PathPlanner::driveStraightLine(){
	double dist_inc = 0.5;
    for(int i = 0; i < 50; i++)
    {
          next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
          next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
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

	double dist_inc = 0.5;
	for(int i = 0; i < 50-path_size; i++)
	{    
	  next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(M_PI/100)));
	  next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(M_PI/100)));
	  pos_x += (dist_inc)*cos(angle+(i+1)*(M_PI/100));
	  pos_y += (dist_inc)*sin(angle+(i+1)*(M_PI/100));
	}
}

//Test #3: Follow the current line. Problem: Harsh turns cause high jerks
void PathPlanner::followLane(){
	double dist_inc = 0.4;
	for(int i = 0; i < 50; i++)
    {
    	double next_s = car_s+(i+1)*dist_inc;
    	double next_d = 6;
    	vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }
}

//Test #4: Follow the current line using Splines
void PathPlanner::followLane_with_Splines(){
   
   followLane();
   
   //Define 3 anchor points for calculating smooth spline

   double anchorPoint_past_x;
   double anchorPoint_past_y;

   if(previous_path_x.size()>1){
	   anchorPoint_past_x = previous_path_x[0];
	   anchorPoint_past_y = previous_path_y[0];
   }
   else{
   	   anchorPoint_past_x = next_x_vals[0];
   	   anchorPoint_past_y = next_y_vals[0];
   }

   double anchorPoint_middle_x = next_x_vals[next_x_vals.size()/2-1];
   double anchorPoint_middle_y = next_y_vals[next_y_vals.size()/2-1];

   double anchorPoint_end_x = next_x_vals[next_x_vals.size()-1];
   double anchorPoint_end_y = next_y_vals[next_y_vals.size()-1];

   vector<double> anchorPoints_x(3);
   anchorPoints_x = {anchorPoint_past_x, anchorPoint_middle_x, anchorPoint_end_x};

   vector<double> anchorPoints_y(3);
   anchorPoints_y = {anchorPoint_past_y, anchorPoint_middle_y, anchorPoint_end_y};
   
   tk::spline s;
   s.set_points(anchorPoints_x,anchorPoints_y);

   for(int i=0;i<next_x_vals.size();i++){
   		next_y_vals[i] = s(next_x_vals[i]);
   }
}
