#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {

public:

  int vid;

  bool change_lane = false;

  // car length
  int L = 4;

  // position of vehicle
  double pos_x, pos_y, angle, pos_s, pos_d;

  // horizontal and vertical speed, needed for the other vehicles
  double vx,vy;

  // minimum distance ego of other vehicles for same and adjacent lanes
  double other_min_s_same_lane, other_min_s_left_lane, other_min_s_right_lane;

  // lane of the vehicle 0,1,2
  int lane;

  // several speeds for ego vehicle
  double speed, target_speed, front_vehicle_speed;
  double max_speed = 49.0;

  const double acceleration = 0.224; // miles/s/s

  // trajectory points
  vector<double> next_x_vals, next_y_vals;

  /**
  * Constructor
  */
  Vehicle(int vid);

  /**
  * Destructor
  */
  virtual ~Vehicle();


  void UpdateState(double pos_x, double pos_y, double angle, double pos_s, double pos_d);

  void UpdateTrajectory(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                                vector<double> previous_path_x, vector<double> previous_path_y);


  void UpdateOtherState(vector<double> state);

  void UpdateSpeed();

  void LaneChange(int lane);

  void CenterofLaneCheck();

};

#endif
