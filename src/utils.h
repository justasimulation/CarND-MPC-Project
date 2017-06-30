#ifndef MPC_UTILS_H
#define MPC_UTILS_H

#include <stdlib.h>
#include <math.h>
#include <string>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

/**
 * Converts speed in miles per hour to meters per second
 * @param mph - speed in miles per hour
 * @return speed in meters per second
 */
double mileshour2meterssecond(double mph);

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s);

#endif //MPC_UTILS_H
