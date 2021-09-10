#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// Initialization - define macros
# define mps2mph 2.24 // factor from m/s to mph
# define TS 0.02 // sampling timestamp
# define MAX_SPEED 49.5// tighter velocity threshold for 50mph limit
# define MAX_ACC 9.9 // tighter acceleration threshold for 10m/s^2
# define LANE_WIDTH 4.0
# define WAYPOINT_STEP_S 30.0
# define LANENUM_DEFAULT 1 // defined default lane -- where host vehicle should stay unless changing lanes
# define LANENUM_LEFTMOST 0
# define LANENUM_RIGHTMOST 2
# define MAX_PATH_SIZE 50

int findlane(double d, double lane_width) {
  // return lane # given Frenet-d and lane width
  // assume d>0
  return floor(d/lane_width);
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
  
  // Initialization

  int lane = LANENUM_DEFAULT; // host car starts from default lane
  double ref_vel = 0.0; // mph. Host car is stopped at the beginning
  double max_delta_vel = MAX_ACC*TS; // ref_vel update step upon given TS (0.02s)

  h.onMessage([&ref_vel, &max_delta_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // Parse sensor fusion: get information of other cars
          // sensor fusion format: id,x,y,vx,vy,s,d
          bool close_front = false;
          bool close_left = false;
          bool close_right = false;
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            float d = sensor_fusion[i][6];
            int car_lane = findlane(d, LANE_WIDTH);
            if ((car_lane >= 0) && (abs(car_lane-lane)<=1)) {
              // proceed only if the detected car is 
              // i)  in the same direction, and
              // ii) in the host lane or adjacent lane
              
              // Find car statuses
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              // Estimate car s position by projecting with moving increment (step k-1 -> k)
              check_car_s += ((double)prev_size*TS*check_speed);
              
              // Existing vehicle ahead and too close 
              // use OR logic -- true if ANY car is close
              close_front |= ((car_lane == lane) && (check_car_s > car_s) && (check_car_s < car_s+WAYPOINT_STEP_S));
              close_left |= ((car_lane == lane-1) && (check_car_s > car_s-WAYPOINT_STEP_S) && (check_car_s < car_s+WAYPOINT_STEP_S));
              close_right |= ((car_lane == lane+1) && (check_car_s > car_s-WAYPOINT_STEP_S) && (check_car_s < car_s+WAYPOINT_STEP_S));
            }
          }
          
          // Behavior
          double ref_vel_delta = 0.0;
          if (close_front) {
            // Vehicle close in front:
            // i)  change lane if safe to do so, or
            // ii) slow down
            if ((lane!=LANENUM_LEFTMOST) && (!close_left)) {
              lane--;
            } else if ((lane!=LANENUM_RIGHTMOST) && (!close_right)) {
              lane++;
            } else {
              ref_vel_delta = -max_delta_vel;
            }
          } else {
            // If vehicle is not in default lane, change towards the default lane (if safe to do so)
            if ((lane > LANENUM_DEFAULT) && (!close_left)) {
              lane--;
            } else if ((lane < LANENUM_DEFAULT) && (!close_right)) {
              lane++;
            }
            // Accelerate if too slow
            if (ref_vel < MAX_SPEED) {
              // speed up when too slow
              ref_vel_delta = max_delta_vel;
            }
          }
//          // referal velocity must be positive and within speed limit
//          ref_vel = (ref_vel > MAX_SPEED) ? MAX_SPEED : ref_vel;
//          ref_vel = (ref_vel < MAX_ACC) ? MAX_ACC : ref_vel;
              
          // create a list of widely spaced (x,y) waypoints
          // They consist of past and future points, which will be used as accordance for spline interpolation
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Set up reference x,y,raw states
          // either current car states or prev path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // 5 way-points
          // Way-points 1~2: push last 2 points of prev path into waypoints, if it is possible
          if ( prev_size < 2 ) {
          	// if prev path has < 2 points, use current car position and calculated "prev" position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Way-points 3~5: set up 3 target points in the future (30/60/90m in s-direction).
          double car_d_lc = ((double) lane + 0.5) * LANE_WIDTH; // lane center d position
          vector<double> next_wp0 = getXY(car_s + WAYPOINT_STEP_S, car_d_lc, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 2*WAYPOINT_STEP_S, car_d_lc, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 3*WAYPOINT_STEP_S, car_d_lc, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Make coordinates to local car coordinates.
          // ref point local: x = 0, y = 0, yaw angle = 0
          for ( int i = 0; i < ptsx.size(); i++ ) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
          
          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);
          
          // Output path points from previous path for continuity.
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate distance y position on 30 m ahead.
          double target_x = WAYPOINT_STEP_S;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          // Fill up the rest points (0.02s interval) between way points
          double x_add_on = 0;
          for( int i = 1; i < MAX_PATH_SIZE - prev_size; i++ ) {
            // consider ref_vel's actual change per point
            ref_vel += ref_vel_delta;
            // referal velocity must be positive and within speed limit
            ref_vel = (ref_vel > MAX_SPEED) ? MAX_SPEED : ref_vel;
            ref_vel = (ref_vel < max_delta_vel) ? max_delta_vel : ref_vel;
            
            // calculate xy of each point
            double N = target_dist/(TS*ref_vel/mps2mph);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            
            // convert local points to real-world cartesian coordinate
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }           
          // END
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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