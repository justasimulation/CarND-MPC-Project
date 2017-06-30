#ifndef MPC_CONFIG_H
#define MPC_CONFIG_H

#include "utils.h"

//Contains presents for different maximum speeds.
//Maximum is speed is reflected in class name e.g. Config60 (max 60 mph)

class Config
{
public:
    static Config Instance;

    // Time delta that is used in motion equations
    // actually I think we need to calculate time delta based on time between simulator calls
    // as it would be an accurate modelling of reality. I tried it but for some reasons results
    // became worse. So I decided to stick with a working solution.
    double dt;

    // Target velocity
    double target_v;

    // Weight of cross track error
    double cte_w;

    // Weight of steering angle error
    double epsi_w;

    // Weight of velocity being different from its target
    double velocity_diff_w;

    // Weight of steering angle magnitude
    double delta_w;

    // Weight of acceleration magnitude
    double a_w;

    // Weight of sequential steering actions being different (not smooth)
    double delta_diff_w;

    // Weight of sequentioal acceleration actions being different (not smooth)
    double a_diff_w;

    // Maximum cpu time for optimizer. Not sure what it means or works at all.
    double max_cpu_time;

    // Maximum number of points in the predicted trajectory.
    int max_points_num;

    static Config GetConfig()
    {
        return Config::Instance;
    }
};

class Config50 : public Config
{
public:
    Config50()
    {
        dt = 0.05;
        target_v = mileshour2meterssecond(50);
        cte_w = 1;
        epsi_w = 100;
        velocity_diff_w = 1000;
        delta_w = 1;
        a_w = 1;
        delta_diff_w = 5000;
        a_diff_w = 5000;
        max_cpu_time = 0.05;
        max_points_num = 30;
    }
};


class Config60 : public Config
{
public:
    Config60()
    {
        dt = 0.05;
        target_v = mileshour2meterssecond(60);
        cte_w = 1;
        epsi_w = 120;
        velocity_diff_w = 1000;
        delta_w = 1;
        a_w = 1;
        delta_diff_w = 7000;
        a_diff_w = 7000;
        max_cpu_time = 0.05;
        max_points_num = 30;
    }
};

class Config70 : public Config
{
public:
    Config70()
    {
        dt = 0.2;
        target_v = mileshour2meterssecond(70);
        cte_w = 1;
        epsi_w = 250;
        velocity_diff_w = 1000;
        delta_w = 1;
        a_w = 1;
        delta_diff_w = 20000;
        a_diff_w = 7000;
        max_cpu_time = 0.5;
        max_points_num = 9;
    }
};


#endif //MPC_CONFIG_H
