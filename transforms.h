#ifndef TRANSFORMS_H    
#define TRANSFORMS_H

double deg2rad(double x);

double rad2deg(double x);

vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

int DtoLane(double d, double lane_width);

#endif
