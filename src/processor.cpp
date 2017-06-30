#include <iostream>
#include <thread>

#include "processor.h"
#include "MPC.h"

int Processor::CalcPointsNum(vector<double> &x, vector<double> &y, double v, double dt, int max_points_num)
{
    // calculate number of points in the predicted trajectory.
    // the trajectory should not be longer than the target line,
    // otherwise optimizer produces bad results
    // so we're going to calculate number of points so the result trajectory will be approximately
    // the same length as the target trajectory.

    double length = 0;
    for(size_t i = 1; i < x.size(); i++)
    {
        double dx = x[i] - x[i-1];
        double dy = y[i] - y[i-1];
        length += sqrt(dx * dx + dy * dy);
    }

    double distance_per_dt = v * dt;
    int points_num = distance_per_dt > 0.05 ? (int)(length / distance_per_dt) : max_points_num;

    return min(points_num, max_points_num);
}

Response Processor::Process(vector<double> &pts_x, vector<double> &pts_y,
                            double px, double py, double psi, double v,
                            double throttle, double steering_angle)
{
    // 1. Get start time to measure internal execution time and time between method calls
    double start_time = GetTimeS();

    // 2. Calculate time between the method calls
    double time_delta = prev_time < 0 ? 0.1 : start_time - prev_time;
    prev_time = start_time;
    av_iteration_time = AddToEMA(av_iteration_time, time_delta);

    // 3. convert speed from mile/h to m/s
    v = mileshour2meterssecond(v);

    // 4. calculate current acceleration in m/s**2
    double acceleration = (v - prev_speed) / time_delta;
    prev_speed = v;

    // 5. calculate number of points in the predicted trajectory
    Config config = Config::GetConfig();
    int points_num = CalcPointsNum(pts_x, pts_y, v, config.dt, config.max_points_num);

    // 6. handle latency
    // we consider latecy as the average execution time of this method
    // and apply motion equations to the current state given this time
    double latency = av_local_processing_time;
    px = px + v * cos(psi) * latency;
    py = py + v * sin(psi) * latency;
    psi = psi - v * steering_angle / Lf * latency;
    v = v + acceleration * latency;

    Response response;
    Eigen::VectorXd xs(pts_x.size());
    Eigen::VectorXd ys(pts_y.size());
    Eigen::VectorXd state(6);

    // 7. Convert waypoints to the new car's coordinates system keeping in mind latency
    for(size_t i = 0; i < pts_x.size(); i++)
    {
        double new_x = pts_x.at(i) - px;
        double new_y = pts_y.at(i) - py;
        double x = new_x * cos(psi) + new_y * sin(psi);
        double y = -new_x * sin(psi) + new_y * cos(psi);

        xs[i] = x;
        ys[i] = y;

        // also fill response waypoints
        response.x_car_waypoints.push_back(x);
        response.y_car_waypoints.push_back(y);
    }

    // 8. Fit 3d order polynomial to the waypoints so it is in cars predicted coordinate system.
    auto coeffs = polyfit(xs, ys, 3);


    // 9. Let optimizer find the solution to this polynomial trajectory given number of points
    state << 0., 0., 0., v, coeffs[0], atan(-coeffs[1]);
    MPC mpc;
    auto solution = mpc.Solve(state, coeffs, points_num);

    // 10. Fill the results structure

    // Convert steering angle according to simulator rules [-1 == -25 degrees, +1 == 25 degrees]
    response.steering_angle = -solution.delta/ deg2rad(25);

    // Convert acceleration to actuator value from [-4, 4] to [-1, 1]
    // This is a very rough approximation, it assumes that if we give 1 we'll have 4 m/s**2 acceleration,
    // which is almost always is false.
    // We assume that we will resolve the problem often enough so the roughness of this approach
    // will be invisible.
    response.throttle = max(min(solution.acceleration * 0.25, 1.), -1.);

    //Display the MPC predicted trajectory
    response.x_car_trajectory = solution.x_vals;
    response.y_car_trajectory = solution.y_vals;


    // sleep to emulate actuators latency
    this_thread::sleep_for(chrono::milliseconds(100));

    // memorize execution time
    double end_time = GetTimeS();
    av_local_processing_time = AddToEMA(av_local_processing_time, end_time - start_time);

    return response;
}
