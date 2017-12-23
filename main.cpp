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
#include "vehicle.h"
#include "vehicle.cpp"
#include "transforms.h"
#include "transforms.cpp"
#include "road.h"
#include "road.cpp"

using namespace std;

// for convenience
using json = nlohmann::json;


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


int main() {
  uWS::Hub h;

  // Define the landscape
  string map_file_ = "../data/highway_map.csv";
  Road road = Road(map_file_);

  // Define self as vehicle
  Vehicle ego = Vehicle(-1); // ego has id = -1


  h.onMessage([&ego, &road](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

                // Updating the car state with the latest sensor data
                ego.UpdateState(car_x,car_y,car_yaw, car_s,car_d);


                // Updating the other cars information with the latest sensor fusion data
                // and extracting minimum distances across the lanes

                for (int i=0; i < sensor_fusion.size(); i++){
                        int vid = sensor_fusion[i][0];
                        Vehicle otherVehicle = Vehicle(vid);
                        vector<double> v = sensor_fusion[i]; // temporary variable
                        vector<double> state(v.begin() + 1, v.end());
                        otherVehicle.UpdateOtherState(state);

                        double s_dist = otherVehicle.pos_s - ego.pos_s - ego.L;

                        // Updating minimum instances for ego lane, and left and right lanes where possible
                        // For same lane speed is captured in order to syncronize when necessary
                        if(otherVehicle.lane == ego.lane){
                            if(s_dist < ego.other_min_s_same_lane && s_dist > 0){ // we care only about cars in front for same lane
                                ego.other_min_s_same_lane = s_dist;
                                ego.front_vehicle_speed = otherVehicle.speed;
                            }
                        }
                        else if (ego.lane - otherVehicle.lane == +1){ // other car is left
                            if(abs(s_dist) < ego.other_min_s_left_lane){
                                ego.other_min_s_left_lane = abs(s_dist);
                            }
                        }
                        else if (ego.lane - otherVehicle.lane == -1){ // other car is right
                            if(abs(s_dist) < ego.other_min_s_right_lane){
                                ego.other_min_s_right_lane = abs(s_dist);
                            }
                        }
                }


                // Traffic control section

                // Priority is on lane change otherwise adjust speed

                double safe_distance = ego.speed - 18; // safe distance for front vehicle in same lane
                double change_lane_safe_distance = safe_distance + 5; // safe distance of vehicles in adjacent lanes


                // Approaching a slower vehicle on the same lane
                if(ego.other_min_s_same_lane < safe_distance){
                    // check if possible to move on the left lane
                    // if both lanes available choose left

                    if(ego.lane > 0 && ego.other_min_s_left_lane > change_lane_safe_distance && !ego.change_lane){
                        ego.lane -= 1;
                        ego.change_lane = true;
                    }
                    // check if possible to move on the right lane
                    else if(ego.lane <2 && ego.other_min_s_right_lane > change_lane_safe_distance
                             && !ego.change_lane){
                        ego.lane += 1;
                        ego.change_lane = true;
                    }
                    // otherwise adjust speed
                    else {
                    // check is lane change is possible
                    ego.target_speed = ego.front_vehicle_speed;
                    }
                }
                else {
                    ego.target_speed = ego.max_speed;
                }

                // check if a lane transition has been completed
                ego.CenterofLaneCheck();

                // Update the path trajectory
                ego.UpdateTrajectory(road.map_waypoints_x, road.map_waypoints_y,road.map_waypoints_s,
                                     previous_path_x, previous_path_y);


                // Sending the updated trajectory to the simulator
                json msgJson;
                msgJson["next_x"] = ego.next_x_vals;
                msgJson["next_y"] = ego.next_y_vals;

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
