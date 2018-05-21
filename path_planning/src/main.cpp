#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

float inefficiency_cost(float target_speed, int intended_lane, int final_lane, vector<float> lane_speeds)
{
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */
	float speed_intended = lane_speeds[intended_lane];
	float speed_final = lane_speeds[final_lane];
	//float cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;
	float delta = (2.0*target_speed - speed_intended - speed_final);
	float cost = 1-exp(-(abs(delta) /target_speed));
    return cost;
}

int calc_efficient_lane_index(float target_speed, vector<float> lane_speeds, vector<int> lane_collisions)
{
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed.
    */
	vector<float> lane_costs;

	for(unsigned int i = 0; i< lane_speeds.size(); i++ ){
		float speed_intended = lane_speeds[i];
		float delta = (2.0*target_speed - speed_intended);
		float cost = (1-exp(-(abs(delta) /target_speed))) + lane_collisions[i];
		lane_costs.push_back(cost);
	}
	std::cout << std::distance(lane_costs.begin(), std::max_element(lane_costs.begin(), lane_costs.end()))  << endl;
	return std::distance(lane_costs.begin(), std::min_element(lane_costs.begin(), lane_costs.end()));
}

vector<float> calc_lane_costs(float target_speed, vector<float> lane_speeds)
{
    /*
    Cost becomes higher for trajectories with that have traffic slower than target_speed.
    */
	vector<float> lane_costs;

	for(unsigned int i = 0; i< lane_speeds.size(); i++)
	{
		float speed_intended = lane_speeds[i];
		float delta = (target_speed - speed_intended);
		float cost = (1-exp(-(abs(delta) /target_speed))) ;
		lane_costs.push_back(cost);
	}
	return lane_costs;
}


vector<float> calc_lane_costs2(float target_speed, vector<float> lane_speeds, vector<int> targets)
{

	vector<float> lane_costs;

	for(unsigned int i = 0; i< lane_speeds.size(); i++)
	{
		float speed_intended = lane_speeds[i];
		float delta = (target_speed - speed_intended - targets[i]*10);
		float cost = (1-exp(-(abs(delta) /target_speed))) ;
		lane_costs.push_back(cost);
	}
	return lane_costs;
}

/**
 * Returns all feasible lane change maneuvers
 * Current_lane == center lane -->  left + right
 * Current_lane == left most lane -->  right
 * Current_lane == right most lane -->  left
 */
vector<int> get_feasible_lane_change(int current_lane){

	vector<int> feasible_lanes = {0,0};

	// left lane
	feasible_lanes[0] = (current_lane -1) >= 0;

	// right lane
	feasible_lanes[1] = (current_lane +1) <= 2;

	return feasible_lanes;
}

int get_next_lane(vector<float> lane_costs, vector<int> feasible_lanes, vector<int> lane_collisions, int current_lane){

	int next_lane = current_lane;
	// check if left lane change maneuver is feasible
	if(feasible_lanes[0] > 0)
	{
		if(lane_costs[current_lane]>lane_costs[current_lane-1] && lane_collisions[current_lane-1] == 0)
		{
			next_lane = current_lane-1;
		}
	}
	// check if right lane change maneuver is feasible
	// prefer left lane change if EGO is in center lane as long as costs are equal
	if(feasible_lanes[1] > 0)
	{
		if(lane_costs[current_lane]>lane_costs[current_lane+1]
					   && lane_collisions[current_lane+1] == 0
							   	   && lane_costs[current_lane-1] > lane_costs[current_lane+1] )
		{
			next_lane = current_lane+1;
		}
	}

	return next_lane;

}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  //initial lane no.
  int lane = 1;
  // reference velocity
  float ref_vel = 0;
  // target velocity
  float const target_velocity = 49.5;
 // state
  string state = "FOLLOW_LANE";
  // target vehicle velocity
  float target_vehicle_velocity = 0;
  // left lane velocity
  float left_lane_velocity = 0;
  // right lane velocity
  float right_lane_velocity = 0;

  int state_timer = 100;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

             vector<double> ptsx;
             vector<double> ptsy;

             double ref_x = car_x;
             double ref_y = car_y;
             double ref_yaw = deg2rad(car_yaw);

             int prev_size = previous_path_x.size();

             if(prev_size > 0)
             {
            	 	 car_s = end_path_s;
             }

             bool target_vehicle_present = false;
             bool potential_collision_left = false;
             bool potential_collision_right = false;
             vector<float> lane_velocities = {49.5, 49.5, 49.5};
             vector<int> potential_targets = {0,0,0};
             vector<int> lane_collisions = {0, 0, 0};
             vector<int> lane_costs_unclear = {0, 0, 0};

             lane_velocities[lane] = car_speed;

             int collision_zone_left_d = 0;
                     	 		int left_s = 0;
                     	 		int center_d = 0;
                     	 		int right_d = 0;
                     	 		int right_s = 0;
             for(unsigned int i = 0; i < sensor_fusion.size(); i++)
             {
            	 	 /** sensor_fusion explained:
            	 	  * [i][0] = id
            	 	  * [i][1] = x
            	 	  * [i][2] = y
            	 	  * [i][3] = velocity in x direction
            	 	  * [i][4] = velocity in y direction
            	 	  * [i][5] = s
            	 	  * [i][6] = d
            	 	  */

            	 	 // characteristics of dynamic objects
            	 	 float dynamic_object_d = sensor_fusion[i][6];
        	 		 double dynamic_object_vx = sensor_fusion[i][3];
        	 		 double dynamic_object_vy = sensor_fusion[i][4];

        	 		 // get speed using the distance in cartesian coordinates
        	 		 double dynamic_object_vel_meter = sqrt(dynamic_object_vx*dynamic_object_vx+dynamic_object_vy*dynamic_object_vy);
        	 		 float dynamic_object_vel = sqrt(dynamic_object_vx*dynamic_object_vx+dynamic_object_vy*dynamic_object_vy) * 2.24;
        	 		 double dynamic_object_s = sensor_fusion[i][5];

        	 		 // simple extrapolation of car's position (the prediction part)
        	 		 dynamic_object_s = dynamic_object_s + (double)prev_size * 0.02 * dynamic_object_vel_meter;


            	 	 // each lane is 4m
            	 	 // center of each lane is at 2m
            	 	 // the buffer below covers the whole lane

            	 	int left_lane = lane - 1;

            	 	int right_lane = lane + 1;

            	 	// left lane (never true if EGO is in the left most lane)
				if(dynamic_object_d < (4*left_lane+4) && dynamic_object_d > (4*left_lane))
				{
					// TARGET ZONE
					// this car is in my left lane
					//target buffer front +20
					if((dynamic_object_s > (car_s + 10)) && (dynamic_object_s < (car_s + 30)))
					{
						// this car is in front of me (potential target object)
						left_lane_velocity = dynamic_object_vel;
						lane_velocities[left_lane] = dynamic_object_vel;
						potential_targets[left_lane] = 1;
					}
					// COLLISION ZONE
					if((dynamic_object_s > (car_s - 10)) && (dynamic_object_s < (car_s + 15)))
					{
						// this car is behind or next to me
						potential_collision_left = true;
						lane_collisions[left_lane] = 1;
						collision_zone_left_d = dynamic_object_d;
						left_s = dynamic_object_s;
					}
				}

            	 	// right lane (never true if EGO is on the right most lane)
				if(dynamic_object_d < (2+4*right_lane+2) && dynamic_object_d > (2+4*right_lane-2)){

					// TARGET ZONE
					// this car is in my right lane
					if((dynamic_object_s > (car_s + 10)) && (dynamic_object_s < (car_s + 30)))
					{
						// this car is in front of me
						right_lane_velocity = dynamic_object_vel;
						lane_velocities[right_lane] = dynamic_object_vel;
						potential_targets[right_lane] = 1;
					}
					// COLLISION ZONE
					if((dynamic_object_s > (car_s - 10)) && (dynamic_object_s < (car_s + 15)))
					{
					// this car is behind or next to me
					potential_collision_right = true;
					lane_collisions[right_lane] = 1;
					right_d = dynamic_object_d;
					right_s = dynamic_object_s;
					}
				}

            	 	 // EGO lane
            	 	 if(dynamic_object_d < (2+4*lane+2) && dynamic_object_d > (2+4*lane-2))
            	 	 {
            	 		// check if car is in front of EGO and if gap is smaller than 30
            	 		if((dynamic_object_s > (car_s + 10)) && (dynamic_object_s < (car_s + 30)))
            	 		{
            	 			// vehicle in target zone identified
            	 			// set flag
            	 			target_vehicle_present = true;
            	 			target_vehicle_velocity = dynamic_object_vel;
            	 			lane_velocities[lane] = dynamic_object_vel;
            	 			potential_targets[lane] = 1;
            	 		}
            	 	 }
             }


  			vector<float> lane_costs = calc_lane_costs2(49.5, lane_velocities, potential_targets);
			vector<int> possible_lanes = get_feasible_lane_change(lane);
			int next_lane = get_next_lane(lane_costs, possible_lanes, lane_collisions, lane);


			cout << "####### PLANNING INFO #########" << endl;
			cout << "----- Left lane ----" << endl;
			cout << "Cost left lane: " << lane_costs[0] << endl;
			cout << "Potential collision on left lane: " << potential_collision_left << endl;
			cout << "Velocity of left lane: " << lane_velocities[0] << endl;
			cout << "----- Center lane ----" << endl;
			cout << "Cost center lane: " << lane_costs[1] << endl;
			cout << "Velocity of center lane: " << lane_velocities[1] << endl;
			cout << "----- Right lane ----" << endl;
			cout << "Cost right lane: " << lane_costs[2] << endl;
			cout << "Potential collision on right lane: " << potential_collision_right << endl;
			cout << "Velocity of right lane: " << lane_velocities[2] << endl;
            cout << "####### EGO INFO #########" << endl;
            cout << "EGO velocity: " << car_speed << endl;
            cout << "Next state change timer : " << state_timer << endl;
            cout << "Current lane: " << lane << endl;
            cout << "State: " << state << endl;


             if("FOLLOW_LANE" == state){
             	 if(true == target_vehicle_present)
             	 {
             		state = "TRACK_GAP";
             	 }
             	 else
             	 {
             		 if(ref_vel < 49.5)
             		 {
             			 // keep velocity
             		     ref_vel = ref_vel + 0.5;
             		  }
             	 }
             	if(state_timer > 0){
             	    // decrement state timer
             		state_timer= state_timer - 1;
             	}
             }

             else if("TRACK_GAP" == state)
             {
            		if(ref_vel>target_vehicle_velocity)
            		{
            			// decrease velocity
            			ref_vel = ref_vel - 0.3;
            		}
            		else if(ref_vel<target_vehicle_velocity + 5)
            		{
            			// increase velocity in case target vehicle accelerates
            		    ref_vel = ref_vel + 0.3;
            		}

            		if(state_timer > 0){
            			// decrement state timer
            			state_timer= state_timer - 1;
            		}


            		if (lane != next_lane && state_timer<=0)
            		{
            			lane = next_lane;
            			state = "FOLLOW_LANE";
            			target_vehicle_present = false;
            			state_timer= 150;
            		}
             }

             if(prev_size < 2)
             {
            	 	 // not enough points on path to be used
            	 	 // initiate path (starting point is vehicle's pose)

            	 	 // since no previous points are available, create one previous point using the tangent points
            	 	 double prev_car_x = car_x - cos(car_yaw);
            	 	 double prev_car_y = car_y - sin(car_yaw);

            	 	 // store these values into the vector
            	 	 ptsx.push_back(prev_car_x);
            	 	 ptsx.push_back(car_x);

            	 	 ptsy.push_back(prev_car_y);
            	 	 ptsy.push_back(car_y);
            	 	 // --> now you got two x, and y points in the vector
             }
             else
             {
            	 	 ref_x = previous_path_x[prev_size-1];
            	 	 ref_y = previous_path_y[prev_size-1];

            	 	 double ref_x_prev = previous_path_x[prev_size-2];
            	 	 double ref_y_prev = previous_path_y[prev_size-2];
            	 	 ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            	 	 ptsx.push_back(ref_x_prev);
            	 	 ptsx.push_back(ref_x);

            	 	 ptsy.push_back(ref_y_prev);
            	 	 ptsy.push_back(ref_y);

             }
            int dist_between_points = 50;

            //get next 3 points
            vector<double> next_wp0 = getXY(car_s+dist_between_points,
            										(2+4*lane),
            											map_waypoints_s,
														map_waypoints_x,
															map_waypoints_y);

            vector<double> next_wp1 = getXY(car_s+2*dist_between_points,
                        										(2+4*lane),
                        											map_waypoints_s,
            														map_waypoints_x,
            															map_waypoints_y);

            vector<double> next_wp2 = getXY(car_s+3*dist_between_points,
                        										(2+4*lane),
                        											map_waypoints_s,
            														map_waypoints_x,
            															map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            // --> now there are 5 points in the vector

            for(unsigned int i = 0; i < ptsx.size(); i++)
            {
            		double delta_x = ptsx[i] - ref_x;
            		double delta_y = ptsy[i] - ref_y;

          	  	// rotate map relative coordinates into car relative coordinates
          	  	ptsx[i] = delta_x * cos(0 - ref_yaw) - delta_y * sin(0 - ref_yaw);
          	  	ptsy[i] = delta_x * sin(0 - ref_yaw) + delta_y * cos(0 - ref_yaw);
            }

            // define spline
            tk::spline s;

            // assign points to spline
            s.set_points(ptsx,ptsy);

            vector<double> next_x_vals;
			vector<double> next_y_vals;

			// if a previous path exist, use its points and add them to the next path
			// the previous_path_x/_y represents the points on the previous path that have not been reached
			// --> enables smooth transitions between paths
			for(unsigned int i = 0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// derive a constant velocity using evenly distanced spline points
			// constant execution cycle time && ref velocity -> decrease in velocity with increase of distance between points
			//  == the distance of points represents the velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

			// start at car origin
			double x_add_on = 0.0;

			// N = amount of points along the line from car origin until target_dist
			double N = (target_dist/(0.02 * (ref_vel / 2.24)));

			for(int i = 0; i < 50 - previous_path_x.size(); i++)
			{
				// next x point
				double x_point = x_add_on + (target_x) / N;
				// next y point
				double y_point = s(x_point);

				// move to next point
				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// rotate into map coordinates
				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
