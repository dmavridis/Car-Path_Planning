#ifndef ROAD_H
#define ROAD_H

class Road {

// Sets the landscape where the cars are moving

public:

  vector<double> map_waypoints_x;

  vector<double> map_waypoints_y;

  vector<double> map_waypoints_s;

  vector<double> map_waypoints_dx;

  vector<double> map_waypoints_dy;

  vector<int> vehicle_ids;

  int LANE_WIDTH = 4;

  string map_file_;
  /**
  * Constructor
  */
  Road(string map_file_);

  /**
  * Destructor
  */
  virtual ~Road();

};

#endif
