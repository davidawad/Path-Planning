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

// bad way to include funcions
#include "util.cpp"

#define TARGET_SPEED 49.5
#define SPEED_INCREMENT 0.224
#define VEHICLE_LATENCY 0.02

using namespace std;

// for convenience
using json = nlohmann::json;


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


  int lane = 1;         // Start in lane 1 (0, 1, 2) being the lanes
  double ref_vel = 0.0; // reference velocity to target in mph

  h.onMessage([&ref_vel,
        &map_waypoints_x,
        &map_waypoints_y,
        &map_waypoints_s,
        &map_waypoints_dx,
        &map_waypoints_dy,
        &lane] (
          uWS::WebSocket<uWS::SERVER> ws,
          char *data, size_t length,
          uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl
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

          // the last path the car was following
          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          int left_lane = lane - 1;
          int right_lane = lane + 1;

          // look through the sensed information for all i cars detected on the road
          for (int i=0; i < sensor_fusion.size(); i++) {

            // info on a detected car is in same lane
            float d = sensor_fusion[i][6];

            // is there a car in our lane
            if (d < (2 + 4*lane + 2) && (d > 2 + 4*lane - 2)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              // s value of the car agead of us
              double check_car_s = sensor_fusion[i][5];

              // what will the car look like in the future
              check_car_s += (double)prev_size * VEHICLE_LATENCY * check_speed;

              // make sure we're a set distance away from car infront of us
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                too_close = true;
                if (too_close){
                  ref_vel -= SPEED_INCREMENT;
                }

                // variables for determining behavior
                bool safe_flag;
                int target_lane;

                // if the car in front of us is going too slow, we can switch lanes to the left
                if (0 <= left_lane && left_lane <= 2) {
                  safe_flag = true;
                  target_lane = left_lane;

                  // iterate through sensed vehicles to make sure we can safely change lanes
                  for (int j = 0; j < sensor_fusion.size(); j++) {
                    float d_local = sensor_fusion[j][6];

                    if (d_local < (2 + 4*target_lane + 2) && (d_local > 2 + 4*target_lane - 2)) {
                      double vx_local = sensor_fusion[j][3];
                      double vy_local = sensor_fusion[j][4];
                      double check_speed_local = sqrt(vx_local*vx_local + vy_local*vy_local );
                      double check_car_s_local = sensor_fusion[j][5];

                      // future position of this car in our target lane
                      check_car_s_local += (double)prev_size * VEHICLE_LATENCY * check_speed_local;

                      // safe_flag = safe_to_switch(check_car_s_local, car_s, check_speed_local, ref_vel);
                      // is the other car of us, AND will we be too close to them if we change lanes?
                      if ((check_car_s_local > car_s) && ((check_car_s_local - car_s) < 40)) {
                        if (check_speed_local < ref_vel) {
                          safe_flag = false;
                        }
                      }

                      // is the other car behind us and will we be far enough ahead if we change lanes?
                      if ((check_car_s_local < car_s) && ((car_s - check_car_s_local) < 20)) {
                        // are we too close
                        safe_flag = false;
                      }
                    }
                  }
                } // lane change left

                // if the car in front of us is going too slow, we can switch lanes to the right
                if (0 <= right_lane && right_lane <= 2) {
                  safe_flag = true;
                  target_lane = right_lane;

                  // look through the cars around us and make sure it's safe to turn
                  for (int j = 0; j < sensor_fusion.size(); j++) {
                    float d_local = sensor_fusion[j][6];

                    if (d_local < (2 + 4*target_lane +2) && (d_local > 2 + 4*target_lane - 2)) {

                      // get car info
                      double vx_local = sensor_fusion[j][3];
                      double vy_local = sensor_fusion[j][4];
                      double check_speed_local = sqrt(vx_local*vx_local + vy_local*vy_local);
                      double check_car_s_local = sensor_fusion[j][5];

                      check_car_s_local += (double)prev_size * VEHICLE_LATENCY * check_speed_local;

                      // safe_flag = safe_to_switch(check_car_s_local, car_s, check_speed_local, ref_vel);
                      if ((check_car_s_local > car_s) && ((check_car_s_local - car_s) < 40)) {
                        if (check_speed_local < ref_vel){
                          safe_flag = false;
                        }
                      }

                      if ((check_car_s_local < car_s) && ((car_s - check_car_s_local) < 20)) {
                        safe_flag = false;
                      }
                    }
                  }
                } // lane change right


                // speed up and set our new lane
                if (safe_flag) {
                  if (ref_vel < TARGET_SPEED) {
                    ref_vel += SPEED_INCREMENT;
                  }
                  lane = target_lane;
                  left_lane = lane - 1;
                  right_lane = lane + 1;
                }
              }
            }
          }

      // if we're too close, slow down, othwerwise hit the gas
      if (too_close) {
        ref_vel -= SPEED_INCREMENT;
      } else if (ref_vel < TARGET_SPEED) {
        ref_vel += SPEED_INCREMENT;
      }

      json msgJson;

      // List of widely spaced waypoints, will be using this for interpolation using spline
      vector<double> ptsx;
      vector<double> ptsy;

      // Referenced x,y and yaw_rate
      // either we will use the car's current position or previous points and paths
      double ref_x   = car_x;
      double ref_y   = car_y;
      double ref_yaw = deg2rad(car_yaw);

      // if previous path is almost empty, use the car as a starting reference
      if (prev_size < 2) { // we're just starting out
        // use two points that make path tangent to the car
        double pre_car_x = car_x - cos(car_yaw);
        double pre_car_y = car_y - sin(car_yaw);

        ptsx.push_back(pre_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(pre_car_y);
        ptsy.push_back(car_y);
      } else { // we have a previous path

        // use the previous points as starting reference
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make path tangent to previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }

      // push new points that are further along on the road
      for (int i=1; i < 4; i++) {

        // get x and y coordinates based on transformations of {s, d} coordinates.
        vector<double> temp = getXY(car_s + 30*i, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

        ptsx.push_back(temp[0]);
        ptsy.push_back(temp[1]);
      }

      // transform all points to car's local coordinates
      for (int i=0; i < ptsx.size(); i++){
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
      }

      // create a spline
      tk::spline s;

      // add our 5 anchor {x,y} points to the spline
      s.set_points(ptsx, ptsy);

      // define actual points that planner will use
      vector<double> next_x_vals;
      vector<double> next_y_vals;

      // start with all the previous points from last line
      for (int i=0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
      }


      // our "horizon" distance out we want to plan for
      double target_x    = 30.0;
      double target_y    = s(target_x);
      double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

      double x_add_on = 0;

      // add remaining points for path planner
      // NOTE previous path will contain all points that have not been "consumed" meaning we only need to generate the new remaining points
      for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

        // we want to split our trajectory into N points
        // car will visit a point every VEHICLE_LATENCY seconds
        // divide by 2.24 to convert from 5 miles per hour to m/s
        // technically this is 2.2352, but they use 2.24 in the livestream
        double N = (target_dist / (VEHICLE_LATENCY * ref_vel / 2.24));
        // TODO parens?
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);
        x_add_on = x_point;

        const double x_ref = x_point;
        const double y_ref = y_point;

        // rotate back to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
      }

      // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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
