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
void PathPlanner::followLane(bool splinesActivated){

	for(int i = 0; i < FUTURE_PATH_SIZE; i++)
    {
    	double next_s = car_s+(i+1)*DIST_INC;
    	double next_d = 6;
    	vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        next_x_vals.push_back(xy[0]);
        next_y_vals.push_back(xy[1]);
    }

    if(splinesActivated){
    	smooth_with_Splines();
    }
}

void PathPlanner::global2local(){
}

void PathPlanner::local2global(){
	
}
//Test #4: Follow the current line using Splines
void PathPlanner::smooth_with_Splines(){
   
   //Define 3 anchor points for calculating smooth spline

   double anchorPoint_past_x;
   double anchorPoint_past_y;
   
   vector<double> anchorPoints_x;
   vector<double> anchorPoints_y;
   
   double previous_size = previous_path_x.size();

   //Previous anchor points
   if(previous_size>2){
	   //anchorPoints_x.push_back(previous_path_x[0]);				  //First Element x
	   //anchorPoints_x.push_back(previous_path_x[previous_size/2]);  //Middle Element x
	   anchorPoints_x.push_back(previous_path_x[previous_size-1]);  //Last Element x

	   //anchorPoints_y.push_back(previous_path_y[0]);				  //First Element 
	   //anchorPoints_y.push_back(previous_path_y[previous_size/2]);  //Middle Element 
	   anchorPoints_y.push_back(previous_path_y[previous_size-1]);  //Last Element 
   }
   else{
		anchorPoints_x.push_back(next_x_vals[0]);			//First Element	x
		anchorPoints_y.push_back(next_y_vals[0]);			//First Element y
   }
   
   double next_size = next_x_vals.size();

   //Future anchor points
   anchorPoints_x.push_back(next_x_vals[next_size/2]);	//Middle Element x
   anchorPoints_x.push_back(next_x_vals[next_size-1]);	//Last Element x

   anchorPoints_y.push_back(next_y_vals[next_size/2]);	//Middle Element y 
   anchorPoints_y.push_back(next_y_vals[next_size-1]);	//Last Element y


   //Transform from global to local
   double ref_x = previous_path_x[previous_size-1];
   double ref_y = previous_path_y[previous_size-1];

   double ref_x_prev = previous_path_x[previous_size-2];
   double ref_y_prev = previous_path_y[previous_size-2];

   double ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

   for(int i=0;i<anchorPoints_x.size();i++){
	   anchorPoints_x[i] = (anchorPoints_x[i]*cos(0-ref_yaw)-anchorPoints_y[i]*sin(0-ref_yaw)) - ref_x;
	   anchorPoints_y[i] = (anchorPoints_x[i]*sin(0-ref_yaw)-anchorPoints_x[i]*cos(0-ref_yaw)) - ref_y;
   }
/*
   //Calculate Spine
   tk::spline s;
   s.set_points(anchorPoints_x,anchorPoints_y);

   //Calculate equally distributed x-values
   double x_step = anchorPoints_x.back() / next_x_vals.size();
   double x_point = 0;

   //Calculate smoothed y-values from spine, transform back and copy to next_y_val
   for(int i=0;i<next_x_vals.size();i++){		
   		double y_point = s(x_point);		//smoothed y values from spine
		
		//Transform from local to global
   		next_x_vals[i] = (x_point * cos(ref_yaw)-y_point*sin(ref_yaw)) + ref_x;
   		next_y_vals[i] = (x_point * sin(ref_yaw)-y_point*cos(ref_yaw)) + ref_y;

   		x_point += x_step;
   }
*/
   cout << "ANKER_X: ";
   for (auto i = anchorPoints_x.begin(); i != anchorPoints_x.end(); ++i)
   		cout << *i << ' ';

   cout << "\nANKER_y: ";
   for (auto i = anchorPoints_y.begin(); i != anchorPoints_y.end(); ++i)
   		cout << *i << ' ';

   cout << "\nCar_x:" << car_x << "    car_y:" << car_y << "    car_yaw:" << car_yaw << "\n\n";
   
}

//ToDo 
//1. Lokale Koordinaten
//2. Smoothing verbessern (z.B. indem du x Koordinaten nochmal sauber rechnest)
//3. FSM
