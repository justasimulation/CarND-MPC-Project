#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include "utils.h"
#include "config.h"
#include "indices.h"
#include "FG_eval.h"

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

MPCSolution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, int points_num)
{
    //indices depend on number of points to fit
    Indices idx(points_num);

    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // number of variables
    size_t n_vars = (idx.N * 6) + ((idx.N - 1) * 2);

    // number of constraints
    size_t n_constraints = idx.N * 6;

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (size_t i = 0; i < idx.delta_start; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (size_t i = idx.delta_start; i < idx.a_start; i++)
    {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.

    // The maximum acceleration that I observed on my pc was about 4 meters per second per second,
    // so acceleration will be +- 4 m/s**2
    // This is not an actuator value this is a real acceleration.
    for (size_t i = idx.a_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -4.0;
        vars_upperbound[i] = 4.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[idx.x_start] = x;
    constraints_lowerbound[idx.y_start] = y;
    constraints_lowerbound[idx.psi_start] = psi;
    constraints_lowerbound[idx.v_start] = v;
    constraints_lowerbound[idx.cte_start] = cte;
    constraints_lowerbound[idx.epsi_start] = epsi;

    constraints_upperbound[idx.x_start] = x;
    constraints_upperbound[idx.y_start] = y;
    constraints_upperbound[idx.psi_start] = psi;
    constraints_upperbound[idx.v_start] = v;
    constraints_upperbound[idx.cte_start] = cte;
    constraints_upperbound[idx.epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs, idx);
    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          " + std::to_string(Config::GetConfig().max_cpu_time) + "\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars,
                                          vars_lowerbound, vars_upperbound,
                                          constraints_lowerbound, constraints_upperbound,
                                          fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;


    MPCSolution sl;
    sl.acceleration = solution.x[idx.a_start];
    sl.delta = solution.x[idx.delta_start];
    // exclude last point because it is a car position, we don't need it
    for(size_t i = 0; i < idx.N - 1; i++)
    {
        sl.x_vals.push_back(solution.x[idx.x_start + 1 + i]);
        sl.y_vals.push_back(solution.x[idx.y_start + 1 + i]);
    }

    return sl;
}
