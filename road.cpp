#include <iostream>
#include <string>
#include "road.h"

Road::Road(string map_file_) {


    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    double lane_width = 4;

    ifstream in_map_(map_file_.c_str(), ifstream::in);
    string line;
    // Reading and storing the map points
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
          this->map_waypoints_x.push_back(x);
          this->map_waypoints_y.push_back(y);
          this->map_waypoints_s.push_back(s);
          this->map_waypoints_dx.push_back(d_x);
          this->map_waypoints_dy.push_back(d_y);
    }
    cout << "Created Landscape" << endl;
}

Road::~Road() {}





