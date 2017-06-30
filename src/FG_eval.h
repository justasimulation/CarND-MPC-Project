#ifndef MPC_FG_EVAL_H
#define MPC_FG_EVAL_H

#include <vector>

#include "indices.h"
#include "config.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//This class is passed to the optimizer and is used to set cost and constraints.
// Implementation as in the lab.
class FG_eval
{
    public:
        // Fitted polynomial coefficients
        Eigen::VectorXd coeffs;
        Indices &idx;
        FG_eval(Eigen::VectorXd coeffs, Indices &idx) : coeffs(coeffs), idx(idx) {}

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector& fg, const ADvector& vars)
        {
            // TODO: implement MPC
            // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
            // NOTE: You'll probably go back and forth between this function and
            // the Solver function below.

            // The cost is stored is the first element of `fg`.
            // Any additions to the cost should be added to `fg[0]`.
            fg[0] = 0;

            Config config = Config::GetConfig();
            // The part of the cost based on the reference state.
            for (size_t t = 0; t < idx.N; t++)
            {
                fg[0] += config.cte_w * CppAD::pow(vars[idx.cte_start + t], 2);
                fg[0] += config.epsi_w * CppAD::pow(vars[idx.epsi_start + t], 2);
                fg[0] += config.velocity_diff_w * CppAD::pow(vars[idx.v_start + t] - config.target_v, 2);
            }

            // Minimize the use of actuators.
            for (size_t t = 0; t < idx.N - 1; t++)
            {
                fg[0] += config.delta_w * CppAD::pow(vars[idx.delta_start + t], 2);
                fg[0] += config.a_w * CppAD::pow(vars[idx.a_start + t], 2);
            }

            // Minimize the value gap between sequential actuations.
            for (size_t t = 0; t < idx.N - 2; t++)
            {
                fg[0] += config.delta_diff_w * CppAD::pow(vars[idx.delta_start + t + 1] - vars[idx.delta_start + t], 2);
                fg[0] += config.a_diff_w * CppAD::pow(vars[idx.a_start + t + 1] - vars[idx.a_start + t], 2);
            }

            //
            // Setup Constraints
            //
            // NOTE: In this section you'll setup the model constraints.

            // Initial constraints
            //
            // We add 1 to each of the starting indices due to cost being located at
            // index 0 of `fg`.
            // This bumps up the position of all the other values.
            fg[1 + idx.x_start] = vars[idx.x_start];
            fg[1 + idx.y_start] = vars[idx.y_start];
            fg[1 + idx.psi_start] = vars[idx.psi_start];
            fg[1 + idx.v_start] = vars[idx.v_start];
            fg[1 + idx.cte_start] = vars[idx.cte_start];
            fg[1 + idx.epsi_start] = vars[idx.epsi_start];


            // The rest of the constraints
            for (size_t t = 1; t < idx.N; t++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[idx.x_start + t];
                AD<double> y1 = vars[idx.y_start + t];
                AD<double> psi1 = vars[idx.psi_start + t];
                AD<double> v1 = vars[idx.v_start + t];
                AD<double> cte1 = vars[idx.cte_start + t];
                AD<double> epsi1 = vars[idx.epsi_start + t];

                // The state at time t.
                AD<double> x0 = vars[idx.x_start + t - 1];
                AD<double> y0 = vars[idx.y_start + t - 1];
                AD<double> psi0 = vars[idx.psi_start + t - 1];
                AD<double> v0 = vars[idx.v_start + t - 1];
                AD<double> cte0 = vars[idx.cte_start + t - 1];
                AD<double> epsi0 = vars[idx.epsi_start + t - 1];

                // Only consider the actuation at time t.
                AD<double> delta0 = vars[idx.delta_start + t - 1];
                AD<double> a0 = vars[idx.a_start + t - 1];

                AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * x0 * x0) + (coeffs[3] * x0 * x0 * x0);
                AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3] * x0 * x0));

                // Here's `x` to get you started.
                // The idea here is to constraint this value to be 0.
                //
                // Recall the equations for the model:
                // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
                // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
                // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                // v_[t+1] = v[t] + a[t] * dt
                // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
                // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
                fg[1 + idx.x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * config.dt);
                fg[1 + idx.y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * config.dt);
                fg[1 + idx.psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * config.dt);
                fg[1 + idx.v_start + t] = v1 - (v0 + a0 * config.dt);
                fg[1 + idx.cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * config.dt));
                fg[1 + idx.epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * config.dt);
            }
        }
};


#endif //MPC_FG_EVAL_H
