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
	/*
	cout << "car_x: " << car_x << endl;
	for(int i=0;i<previous_path_x.size();i++){
		cout << "ausx_previous_path_" << i << ": " << x[i] << endl;
	}
	for(int i=0;i<next_x_vals.size();i++){
		cout << "next_x_vals_" << i << "   x: " << next_x_vals[i] << "    y: " << next_y_vals[i] << endl;
	}*/
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


   //Define anchor points for calculating smooth spline
   vector<double> anchorPoints_x;
   vector<double> anchorPoints_y;

   double previous_size = previous_path_x.size();

   double ref_x;
   double ref_y;
   double ref_yaw;

   double prev_car_x;
   double prev_car_y;

   //if(previous_size<2){
   		//Not enough previous values, so take the current ones
		ref_yaw = deg2rad(car_yaw);

		ref_x = car_x;
   		ref_y = car_y;
 
   		prev_car_x = ref_x - cos(car_yaw);
   		prev_car_y = ref_y - sin(car_yaw);
   	/*}
   	else{
		//Using previous values for stability reasons
   		ref_x = previous_path_x[previous_size-1];
   		ref_y = previous_path_y[previous_size-1];
   		
   		prev_car_x = previous_path_x[previous_size-2];
   		prev_car_y = previous_path_y[previous_size-2];

	    ref_yaw = atan2(ref_y-prev_car_y, ref_x-prev_car_x);
   	}*/

   	//Add anchor points from last 2 known historical points
	anchorPoints_x.push_back(prev_car_x);
	anchorPoints_x.push_back(ref_x);

	anchorPoints_y.push_back(prev_car_y);
	anchorPoints_y.push_back(ref_y);

	//Future Paths
	anchorPoints_x.push_back(next_x_vals[next_x_vals.size()/2]);
	anchorPoints_x.push_back(next_x_vals[next_x_vals.size()-1]);

	anchorPoints_y.push_back(next_y_vals[next_y_vals.size()/2]);
	anchorPoints_y.push_back(next_y_vals[next_y_vals.size()-1]);

	//Transform from global to local car coordinates
	for(int i=0;i<anchorPoints_x.size();i++){
		//Shift 
	    double shift_x = anchorPoints_x[i]-ref_x;
	    double shift_y = anchorPoints_y[i]-ref_y;
		cout << "Before: anchorPoints" << i << ": " << anchorPoints_x[i] << ", " << anchorPoints_y[i] << endl;

		//Rotate
		anchorPoints_x[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
		anchorPoints_y[i] = (shift_x*sin(0-ref_yaw)-shift_y*cos(0-ref_yaw));
		cout << "anchorPoints" << i << ": " << anchorPoints_x[i] << ", " << anchorPoints_y[i] << endl;
	}

	//Calculate Spine
	tk::spline s;
	s.set_points(anchorPoints_x,anchorPoints_y);

	//Calculate equally distributed x-values
	double x_step = anchorPoints_x.back() / next_x_vals.size();
	double x_point = 0;

	//Calculate smoothed y-values from spine, transform back and copy to next_y_val
	for(int i=0;i<next_x_vals.size();i++){		
		double y_point = s(x_point);		//smoothed y values from spine

	//Transform from local car coordinates to global
		//Rotate
		next_x_vals[i] = (x_point * cos(ref_yaw)-y_point*sin(ref_yaw)) + ref_x;
		next_y_vals[i] = (x_point * sin(ref_yaw)-y_point*cos(ref_yaw)) + ref_y;

		//Shift
		x_point += x_step;
		//cout << "x_point: " << x_point << endl;
	}

	cout << "\nCar_x:" << car_x << "    car_y:" << car_y << "    car_yaw:" << car_yaw << "\n\n";
	
}

//ToDo 
//1. Lokale Koordinaten
//2. Smoothing verbessern (z.B. indem du x Koordinaten nochmal sauber rechnest)
//3. FSM
