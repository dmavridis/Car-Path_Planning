#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "spline.h"
#include "road.h"
#include "transforms.h"
/**
 * Initializes Vehicle
 */


Vehicle::Vehicle(int vid) {
    this->vid = vid;
    if(vid==-1)
    {
        cout << "Created ego vehicle " << endl;
        this->speed = 0;
        this->target_speed = this->max_speed;
        this->lane = 1;
        this->other_min_s_left_lane = 1000;  // absolute value
        this->other_min_s_right_lane = 1000; // absolute value
        this->other_min_s_same_lane = 1000; // only front vehicle check
    }
}

Vehicle::~Vehicle() {}

void Vehicle::UpdateState(double pos_x, double pos_y, double angle, double pos_s, double pos_d)
    {
    // Updates the position of ego car with new measurement data
    this->pos_x = pos_x;
    this->pos_y = pos_y;
    this->angle = angle;
    this->pos_s = pos_s;
    this->pos_d = pos_d;

    // recalculate distances
    this->other_min_s_left_lane = 1000;  // absolute value
    this->other_min_s_right_lane = 1000; // absolute value
    this->other_min_s_same_lane = 1000; // only front vehicle check
}

void Vehicle::UpdateTrajectory(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                              vector<double> previous_path_x, vector<double> previous_path_y){

    int lane_width = 4;
    int prev_size = previous_path_x.size();

    // Identify the next closest waypoint

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = this->pos_x;
    double ref_y = this->pos_y;
    double ref_yaw = deg2rad(this->angle);

    this->UpdateSpeed();

    if(prev_size < 2)
    {
        double prev_car_x = this->pos_x - cos(this->angle);
        double prev_car_y = this->pos_y - sin(this->angle);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(this->pos_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(this->pos_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }


    vector<double> next_wp0 = getXY(this->pos_s+45, (2+lane_width*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(this->pos_s+70, (2+lane_width*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(this->pos_s+90, (2+lane_width*this->lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);


    for(int i=0; i < ptsx.size(); i++){

       double shift_x = ptsx[i] - ref_x;
       double shift_y = ptsy[i] - ref_y;

       ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
       ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

    tk::spline s;

    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for(int i=0; i< previous_path_x.size(); i++)
    {
       next_x_vals.push_back(previous_path_x[i]);
       next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 50.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x+target_y*target_y);

    double x_add_on = 0;

   // Update the path buffer
   for(int i = 1; i <= 50 - previous_path_x.size(); i++)
   {
      double N = target_dist/(0.02*this->speed/2.24);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
   }

   this->next_x_vals = next_x_vals;
   this->next_y_vals = next_y_vals;
}

void Vehicle::UpdateSpeed(){
    // Adjust the speed to the desired targer
    if(this->speed < this->target_speed)
    {
        this->speed += this->acceleration;
    }
    else
    {
        this->speed -= this->acceleration;
    }
}


void Vehicle::LaneChange(int lane){
    // check if change is legal, it has to be 0,1,2 and agjacent to current lane
    if(abs(this->lane - lane == 1)){
        this->lane = lane;
    }
}

void Vehicle::CenterofLaneCheck(){
    // checks if car is in the center of the lane, needed to avoid changing lanes during a lane change
    if (fmod(this->pos_d , 4.0) < 2.2 && fmod(this->pos_d , 4.0) > 1.8){
        this->change_lane = false;
    }
}


void Vehicle::UpdateOtherState(vector<double> state){
    // Updates the state of a car, position and velocity
    // Used for the other cars than the ego
    this->pos_x = state[0];
    this->pos_y = state[1];
    this->vx = state[2];
    this->vy = state[3];
    this->pos_s = state[4];
    this->pos_d = state[5];
    this->lane = DtoLane(this->pos_d, 4);
    this->speed = sqrt(this->vx*this->vx + this->vy*this->vy);
}
