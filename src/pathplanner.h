#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include "json.hpp"
#include "helper.h"
#include "pathplanner.h"
#include <vector>


using json = nlohmann::json;
using namespace std;

class PathPlanner
{
    public:
        PathPlanner();
        void setParameters(vector<double>& x,vector<double>& y,vector<double>& s,vector<double>& dx,vector<double>& dy);
        void extractParametersFromJson(json j);

        void driveStraightLine();
        void driveCircles();
        void followLane(bool splinesActivated);  
        void smooth_with_Splines();
        
        vector<double> get_next_x_vals();
        vector<double> get_next_y_vals();

    private:   	
      	//waypoints
      	void global2local();
      	void local2global();

      	const double DIST_INC = 0.4; 
      	const int FUTURE_PATH_SIZE = 50;
      	const double ref_vel = 49;

		vector<double> map_waypoints_x;
		vector<double> map_waypoints_y;
		vector<double> map_waypoints_s;
		vector<double> map_waypoints_dx;
		vector<double> map_waypoints_dy;

    	double car_x;
		double car_y;
		double car_s;
		double car_d;
		double car_yaw;
		double car_speed;

		//ToDo
		// Previous path data given to the Planner
		vector<double> previous_path_x;
		vector<double> previous_path_y;

		// Previous path's end s and d values 
		double end_path_s;
		double end_path_d;

		//ToDo
		// Sensor Fusion Data, a list of all other cars on the same side of the road.
		//auto sensor_fusion = j[1]["sensor_fusion"];

		// Future Path points 
		vector<double> next_x_vals;
      	vector<double> next_y_vals;

};

#endif // PATHPLANNER_H
