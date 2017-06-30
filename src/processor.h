#ifndef MPC_PROCESSOR_H
#define MPC_PROCESSOR_H

#include <chrono>
#include <vector>

#include "config.h"

using namespace std;

// response for telemetry message
class Response
{
    public:
        // way points passed retrieved from telemetry and converted to car's coordinate system
        // this will be shown in yellow
        vector<double> x_car_waypoints;
        vector<double> y_car_waypoints;

        // predicted car trajectory points, this will be shown in green
        vector<double> x_car_trajectory;
        vector<double> y_car_trajectory;

        // steering angle in [-1, 1]
        double steering_angle;

        // throttle in [-1, 1]
        // it is assumed that 1 is around 4 m/s**s, but it is really a rough approximation
        double throttle;
};


//Represents processing of telemetry recieved from simulator
class Processor
{
    private:
        // Last recorded speed. It is needed for acceleration calculation.
        double prev_speed = 0;

        // Last recorded time.
        double prev_time = -1;

        // Average time that it takes to execute Process method.
        double av_local_processing_time = 0.1;

        // Average time span between calls of Process method.
        // I.e. after how much time since calling Process method it will be called again.
        double av_iteration_time        = 0.1;

    public:

        // returns current time in seconds (ms part is shown after the decimal point)
        double GetTimeS()
        {
            auto now = std::chrono::system_clock::now().time_since_epoch();
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
            return now_ms / 1000.;
        }

        // recieves telemetry data and returns actinos and displayed points
        Response Process(vector<double> &pts_x, vector<double> &pts_y,
                         double px, double py, double psi, double v,
                         double throttle, double steering_angle);


        // adds values using exponential moving average
        double AddToEMA(double prev_value, double new_value)
        {
            return prev_value * 0.9 + new_value * 0.1;
        }

        // Calculates number of points in the predicted trajectory so the trajectory is
        // approximately the same length as the target trajectory.
        // Otherwise if we try to fit a trajectory that is much longer that the target line,
        // the optimizer produces bad results.
        int CalcPointsNum(vector<double> &x, vector<double> &y, double v, double dt, int max_points_num);
};

#endif //MPC_PROCESSOR_H
