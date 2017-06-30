#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


class MPCSolution
{
    public:
        double acceleration;
        double delta;
        vector<double> x_vals;
        vector<double> y_vals;
};

// Represents model predictive controller as in the lab
class MPC
{
    public:
        MPC();

        virtual ~MPC();

        // Solve the model given an initial state, polynomial coefficients and number of points to fit.
        // Return the first actuatotions and proposed trajectory points
        MPCSolution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, int points_num);
};

#endif /* MPC_H */
