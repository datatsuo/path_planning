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



// ----------  utility functions used in main() ----------

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; } // from degrees to radians
double rad2deg(double x) { return x * 180 / pi(); } // from radians to degrees

string hasData(string s) {
  /*
  This function checks if the SocketIO event has JSON data.
  If there is data the JSON object in string format will be returned,
  else the empty string "" will be returned.
  */

  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");

  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  /*
  This function computes euclidean distance between
  two points (x1, y1) and (x2, y2)
  (input)
  - x1, y1: Cartesian coordinate of a point
  - x2, y2: Cartesian coordinate of another point
  (output)
  - euclidian distance of the two points
  */
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y){
  /*
  This function returns the index of the closest way point
  to (x, y) on the map.
  (input)
  - x: x-coordinate of the ego car
  - y: y-coordinate of the ego car
  - theta: heading angle of the ego car
  - maps_x: x-coordinates of the way points
  - maps_y: y-coordinates of the way points
  (output)
  index corresponding to the closest waypoint
  */

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for (int i = 0; i < maps_x.size(); i++) {

		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);

		if (dist < closestLen) {
      closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
  /*
  This function returns the index of the next way point.
  (input)
  - x: x-coordinate of the ego car
  - y: y-coordinate of the ego car
  - theta: heading angle of the ego car
  - maps_x: x-coordinates of the way points
  - maps_y: y-coordinates of the way points
  (output)
  index corresponding to the next waypoint
  */

  // get the index of the closest way point to the input (x, y) on the map
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  // (x, y)-coordinate of the closest way points
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

  // heading angle to the closest way point from the input (x, y)
	double heading = atan2((map_y - y),(map_x - x));

  // how much the heading angle to be changed if the vehicle
  // goes to the closest way point in the next time step
	double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  // in case the change of the angle is greater than pi/4,
  // then choose the point with the index = closestWaypoint+1 as the next way point.
  // otherwise, the closestWaypoint is the next way point.
  if (angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y){
  /*
  This function transforms Cartesian coordinates (x,y) to Frenet coordinates (s,d).
  (input)
  - x: x-coordinate of a car
  - y: y-coordinate of a car
  - theta: heading angle of a car
  - maps_x: x-coordinates of the way points
  - maps_y: y-coordinates of the way points
  (output)
  - a vector of Frenet coordinate (s, d)
  */

  // get the next way point
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  // get the previous way point
	int prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0) {
		prev_wp  = maps_x.size() - 1;
	}

	double n_x = maps_x[next_wp] - maps_x[prev_wp]; // difference of x-coordinates for the next way point and previous one
	double n_y = maps_y[next_wp] - maps_y[prev_wp]; // difference of y-coordinates for the next way point and previous one
	double x_x = x - maps_x[prev_wp]; // difference of x and x-coordinate of previous way point
	double x_y = y - maps_y[prev_wp]; // difference of y and y-coordinate of previous way point

	// find the projection of x onto n
	double proj_norm = (x_x * n_x + x_y * n_y)/(n_x * n_x + n_y * n_y);
	double proj_x = proj_norm * n_x;
	double proj_y = proj_norm * n_y;

  // --- calculate s-coordinate ---
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	// see if d value is positive or negative by comparing it to a center point
	double center_x = 1000 - maps_x[prev_wp];
	double center_y = 2000 - maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if (centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// --- calculate s-coordinate ---
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
  /*
  This function transforms Frenet coordinates (s,d) to Cartesian coordinates (x,y).
  (input)
  - s: Frenet s-coordinate of a car
  - d: Frenet d-coordinate of a car
  - maps_s: Frenet s-coordinates of the way points
  - maps_x: x-coordinates of the way points
  - maps_y: y-coordinates of the way points
  (output)
  - a vector of Cartesian coordinates (x, y)
  */

	int prev_wp = -1;

	while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % maps_x.size();

  // heading angle from previous way point to the next one
	double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),(maps_x[wp2] - maps_x[prev_wp]));
	// x,y,s along the segment
	double seg_s = (s - maps_s[prev_wp]);
	double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
	double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - pi()/2; // heading angle - pi/2
	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}

int find_lane(double d){
  /*
  This function identifies the lane number from a given Frenet coordinate d
  (input)
  - d: d-coordinate of the ego car
  (output)
  - lane: the lane number of the ego car
  */

  int lane = -2;

  if(d >= 0 && d < 4){
    lane = 0;
  }
  else if(d >= 4 && d < 8){
    lane = 1;
  }
  else if(d >= 8 && d <= 12){
    lane = 2;
  }

  return lane;
}

vector<bool> nearby_car_checker(int other_lane, double other_s, int lane, double s, double checker_dist){
  /*
  This function check if a given car is located ahead/left/right of the ego car
  (input)
  - other_lane: other car's lane number
  - others_s: other car's s-coordinate
  - lane: lane number of the ego car
  - s: s-coordinate of the ego car
  - checker_dist: distance to check the existence of other cars
  (output)
  - tuple of if the other car is located ahead/on the left/on the right or not
  */

  bool is_ahead = false; // if a vehicle is located within checker_dist ahead or not
  bool is_left = false; // if a vehicle is located on the left or not (within checker_dist ahead and behind)
  bool is_right = false; // if a vehicle is located on the right or not (within checker_dist ahead and behind)

  if(other_lane == lane){
    if( (other_s > s) && (other_s < (s + checker_dist)) ){
      is_ahead = true;
    }
  }
  else if((other_lane+1) == lane){
    if( (other_s > (s - 0.5*checker_dist)) && (other_s < (s + checker_dist)) ){
      is_left = true;
    }
  }
  else if((other_lane-1) == lane){
    if( (other_s > (s - 0.5*checker_dist)) && (other_s < (s + checker_dist)) ){
      is_right = true;
    }
  }

  return {is_ahead, is_left, is_right};
}

double mph2ms(double speed_mph){
  /*
  convert speed in MPH to m/s
  (input)
  - speed_mph: speed in MPH
  (output)
  - speed in m/s
  */
  return 0.44704 * speed_mph;
}

double ms2mph(double speed_ms){
  /*
  convert speed in m/s to MPH
  (input)
  - speed in m/s
  (output)
  - speed in MPH
  */
  return 2.23694 * speed_ms;
}



// ----------  main function ----------
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

  // initialization
  double ref_speed = 1.0; // reference speed
  int car_lane = 1; // lane number of ego car (starting with the center lane)

  h.onMessage([&car_lane, &ref_speed, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    const double dt = 0.02; // time elapse during one time step (in second)
    const double MAX_ACC =  0.44/dt; //maximum acceleration of ego car (in mile/hour/sec, project requires < 10m/s^2 (about 22.36 MPH/s))
    const double MAX_SPEED = 49; // maximum speed of ego car (in MPH, project requires < 50 MPH)
    const int NUM_WAYPOINTS = 50; // number of total way points
    const double LANE_WIDTH = 4.0; // lane width (in meter)
    const double CHECKER_S = 30.0; // distance (in s-coordinate) to check nearby cars

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

        	// Main car's localization data
          	double car_x = j[1]["x"]; // main car's x-coordinate
          	double car_y = j[1]["y"]; // main car's y-coordinate
          	double car_s = j[1]["s"]; // main car's position in s (Frenet coordinate)
          	double car_d = j[1]["d"]; // main car's position in d (Frenet coordinate)
          	double car_yaw = j[1]["yaw"]; // main car's yaw
          	double car_speed = j[1]["speed"]; // main car's speed

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"]; // x-coordinate
          	auto previous_path_y = j[1]["previous_path_y"]; // y-coordinate
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            // the format of each entry is [id, x, y, vx, vy, s, d]
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;


            // in case the previous path way points exit, use the final way point
            // as the location of the ego car at the current time step
            int prev_path_size = previous_path_x.size(); // number of previous path way points
            if(prev_path_size > 0){
              car_s = end_path_s;
              car_d = end_path_d;
            }
            // find the lane number of the ego car
            car_lane = find_lane(car_d);

            /* ----- check if there are any other car on the ahead/left/right or not ----- */

            // if any car located ahead/on the left/on the right of the ego car or not
            vector<bool> other_car_nearby = {false, false, false};
            // numbers of cars located ahead/on the left/on the right of the ego car
            vector<int> other_car_nearby_count = {0, 0, 0};
            // average speeds of cars located ahead/on the left/on the right of the ego car
            vector<double> lane_speed = {0.0, 0.0, 0.0};

            for(int i=0; i < sensor_fusion.size(); i++){

              double other_car_vx = sensor_fusion[i][3];  // in m/s
              double other_car_vy = sensor_fusion[i][4];  // in m/s
              double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy); // in m/s
              double other_car_s = sensor_fusion[i][5]; // Frenet s coordinate of a non-ego car (in meter)
              double other_car_d = sensor_fusion[i][6]; // Frenet d coordinate of a non-ego car (in meter)
              int other_car_lane = find_lane(other_car_d); // lane number of a non-ego car

              other_car_s += dt * other_car_speed * (double)prev_path_size; // take into account the distance the other car advanced during a time step
              // if a given non-ego car is located on the ahead/left/right of the ego car
              vector<bool> is_nearby = nearby_car_checker(other_car_lane, other_car_s, car_lane, car_s, CHECKER_S);

              // update the counts of the nearby cars and lane speeds
              for(int j=0; j < other_car_nearby.size(); j++){

                  other_car_nearby[j] = other_car_nearby[j] | is_nearby[j];
                  if(is_nearby[j]){
                    other_car_nearby_count[j] += 1;
                    lane_speed[j] += other_car_speed;
                  }

              }

            }

            // compute lane speeds as average speeds of the cars located
            // ahead/on the left/on the right of the ego car
            double ref_acc =0.0;
            for(int i=0; i<other_car_nearby_count.size();i++){

              if(other_car_nearby_count[i]!=0){
                lane_speed[i] = ms2mph(lane_speed[i]/((double)other_car_nearby_count[i]));
              }

            }


            /* ----- determine reference speed/acceleration ----- */

            // reference_acceleration
            if(!other_car_nearby[0]){ // in case there is no car in front

                //then add max acceleration so far as the speed is smaller than reference speed
                ref_acc = MAX_ACC;

            }
            else{ // in case there is a car in front

              if(!other_car_nearby[1] && car_lane !=0 ){
                // in case there is a space to move on the left lane, go there
                car_lane -= 1;
                ref_acc =  MAX_ACC;

              }
              else if(!other_car_nearby[2] && car_lane != 2){
                // in case there is a space to move on the right lane, go there
                car_lane += 1;
                ref_acc =  MAX_ACC;

              }
              else{
                if(ref_speed > lane_speed[car_lane]){
                    ref_acc -=  0.8 * MAX_ACC;
                }
              }

            }

            // reference speed
            ref_speed += ref_acc * dt;
            if(ref_speed > MAX_SPEED){
              ref_speed = MAX_SPEED;
            }


            /* ----- create path way points (2 steps) ----- */

            /* --- step 1: use some previous path way points and target poitns to create a spline --- */

            double ref_x; // x-coordinate of the reference point
            double ref_y; // y-coordinate of the reference point
            double ref_angle; // the heading angle at the reference point
            double pos_x_prev; // x-coordinate at one time step before
            double pos_y_prev; // y-coordinate at one time step before

            vector<double> pts_x; // x-coordinates of path points
            vector<double> pts_y; // y-coordinates of path points

            if(prev_path_size > 1){
              // In case there are 2 or more data points in the previous path way points,
              // set the reference position to be the end point of the previous path.
              // The angle is computed from the the final two points of the previous path.
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];
              pos_x_prev = previous_path_x[prev_path_size-2];
              pos_y_prev = previous_path_y[prev_path_size-2];
              ref_angle = atan2((ref_y - pos_y_prev), (ref_x - pos_x_prev)); // in radian

            }
            else{

              // otherwise use the localization data for the reference point
              ref_x = car_x;
              ref_y = car_y;
              ref_angle = deg2rad(car_yaw); //in radian

              // the position at one time step before is determined by subtracting
              // the distance the car can move during one time step assuming that the
              // yaw is given by car_yaw and speed is given by ref_speed
              // Note: cos() and sin() take radian as input
              pos_x_prev = ref_x - dt * ref_speed * cos(ref_angle);
              pos_y_prev = ref_y - dt * ref_speed * sin(ref_angle);

            }

            // save these points
            pts_x.push_back(pos_x_prev);
            pts_y.push_back(pos_y_prev);
            pts_x.push_back(ref_x);
            pts_y.push_back(ref_y);

            // create target points (30m, 60m and 90m ahead in Frenet s-coordinate)
            vector<double> target_pos_xy;
            const double TARGET_S = 30.0; // distance between target points (s-cooridnate, in meter)

            for(int i=0; i < 3; i++){

              double lane_center_d = LANE_WIDTH/2.0 + LANE_WIDTH * (double)car_lane; // d-coordinate of lane center
              // convert to cartesian coordinate and save the points
              target_pos_xy= getXY(car_s + TARGET_S * (double)(i+1), lane_center_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              pts_x.push_back(target_pos_xy[0]);
              pts_y.push_back(target_pos_xy[1]);

            }

            // convert to the car coordinate
            double diff_x; // difference between path point and reference point (x-coordinate)
            double diff_y; // difference between path point and reference point (y-coordinate)

            for(int i=0; i<pts_x.size(); i++){

              diff_x = pts_x[i] - ref_x;
              diff_y = pts_y[i] - ref_y;

              // path points in the car coordinate
              pts_x[i] = diff_x * cos(ref_angle) + diff_y * sin(ref_angle);
              pts_y[i] = - diff_x * sin(ref_angle) + diff_y * cos(ref_angle);

            }

            // create the spline
            tk::spline spl;
            spl.set_points(pts_x, pts_y);

            /* --- step 2: based on the created spline and previous way points,
                                          create and save new path way points --- */

            // store way points in the previous path
            for(int i=0; i<prev_path_size; i++){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // set the x-coordinate of the target point,
            // compute its y-coordinate with spline and distance to the target point
            const double TARGET_X = 30.0;
            double pos_x_target = TARGET_X;
            double pos_y_target = spl(pos_x_target);
            double target_dist = distance(pos_x_target, pos_y_target, 0.0, 0.0);

            // create and save new way points between the reference point and the target point
            diff_x = 0.0; // initialization
            diff_y = 0.0; // initialization
            for(int i=0; i< (NUM_WAYPOINTS - prev_path_size); i++){

                // number of time steps needed to get to the target point
                // (assuming the speed is ref_speed)
                int num_pts = ceil(target_dist/(dt * mph2ms(ref_speed)));

                // how far the ego car moves in the x/y-direction along the spline
                diff_x += pos_x_target/((double)num_pts);
                diff_y = spl(diff_x);

                // map back to the map coordinate from the car coordinate
                double pos_x_next = diff_x * cos(ref_angle) - diff_y * sin(ref_angle);
                double pos_y_next = diff_x * sin(ref_angle) + diff_y * cos(ref_angle);
                pos_x_next += ref_x;
                pos_y_next += ref_y;

                // store new way points
                next_x_vals.push_back(pos_x_next);
                next_y_vals.push_back(pos_y_next);

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
