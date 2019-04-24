#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

using namespace std;

/**
 * Set the timestep length (N) and duration(dt)
 */

// start

size_t N = 10;
double dt = 0.1;

// end





/*
This value assumes the model presented in the classroom is used.

It was obtained by measuring the radius formed by running the vehicle in the
simulator around in a circle with a constant steering angle and velocity on
a flat terrain.

Lf was tuned until the the radius formed by the simulating the model
presented in the classroom matched the previous radius.

This is the length from front to CoG that has a similar radius.
*/

const double Lf = 2.67;


// start

// Init
double g_ref_v    = 100;
double g_ref_cte  = 0;
double g_ref_epsi = 0;

double g_cte_factor        = 1800;
double g_epsi_factor       = 1800;
double g_diff_delta_factor = 160;
double g_delta_factor      = 10;
double g_a_factor          = 10;
double g_diff_a_factor     = 50;

int g_x_start     = 0;
int g_y_start     = g_x_start + N;
int g_psi_start   = g_y_start + N;
int g_v_start     = g_psi_start + N;
int g_cte_start   = g_v_start + N;
int g_epsi_start  = g_cte_start + N;
int g_delta_start = g_epsi_start + N;
int g_a_start     = g_delta_start + N - 1;

// end



class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * Implement MPC:
     * `fg` is a vector of the cost constraints, 
     * `vars` is a vector of variable values (state & actuators)
     *
     * NOTE: You'll probably go back and forth between this function and the Solver function below.
     */


    // start

    ////////////
    // Stage 0: Init Cost Value
    ////////////

    fg[0] = 0;

    for (int i = 0; i < N; i++){

      fg[0] += g_cte_factor * CppAD::pow(vars[g_cte_start + i] - g_ref_cte, 2);
      fg[0] += g_epsi_factor * CppAD::pow(vars[g_epsi_start + i] - g_ref_epsi, 2);
      fg[0] += CppAD::pow(vars[g_v_start + i] - g_ref_v, 2);

    }

    // Update Change Rate
    for (int i = 0; i < N -1; i++){

      fg[0] += g_delta_factor * CppAD::pow(vars[g_delta_start + i], 2);
      fg[0] += g_a_factor * CppAD::pow(vars[g_a_start + i], 2);

    }

    // Update the Sequential Actuations
    for (int i = 0; i < N -2; i++){

      fg[0] += g_diff_delta_factor * CppAD::pow(vars[g_delta_start + i + 1] - vars[g_delta_start + i], 2);
      fg[0] += g_diff_a_factor * CppAD::pow(vars[g_a_start + i + 1] - vars[g_a_start + i], 2);

    }
   


    ////////////
    // Stage 1: Init Model
    ////////////

    fg[g_x_start + 1]    = vars[g_x_start];
    fg[g_y_start + 1]    = vars[g_y_start];
    fg[g_psi_start + 1]  = vars[g_psi_start];
    fg[g_v_start + 1]    = vars[g_v_start];
    fg[g_cte_start + 1]  = vars[g_cte_start];
    fg[g_epsi_start + 1] = vars[g_epsi_start];

    for (int i = 0; i < N-1; i++){

      AD<double> x0    = vars[g_x_start    + i];
      AD<double> y0    = vars[g_y_start    + i];
      AD<double> psi0  = vars[g_psi_start  + i];
      AD<double> v0    = vars[g_v_start    + i];
      AD<double> cte0  = vars[g_cte_start  + i];
      AD<double> epsi0 = vars[g_epsi_start + i];

      AD<double> delta0 = vars[g_delta_start + i];
      AD<double> a0     = vars[g_a_start     + i];

      // Calculate the polynomil
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;

      // Calculate the psides
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

      // Calculate the state at time i + 1
      AD<double> x1    = vars[g_x_start    + i + 1];
      AD<double> y1    = vars[g_y_start    + i + 1];
      AD<double> psi1  = vars[g_psi_start  + i + 1];
      AD<double> v1    = vars[g_v_start    + i + 1];
      AD<double> cte1  = vars[g_cte_start  + i + 1];
      AD<double> epsi1 = vars[g_epsi_start + i + 1];  

      // Add Constraints
      fg[g_x_start    + i + 2] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[g_y_start    + i + 2] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[g_psi_start  + i + 2] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[g_v_start    + i + 2] = v1 - (v0 + a0 * dt);
      fg[g_cte_start  + i + 2] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[g_epsi_start + i + 2] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);

    }


    // end

     
  }
};




/*
MPC class definition implementation.
*/

MPC::MPC() {
}

MPC::~MPC() {
}

std::vector<double> MPC::Solve(const VectorXd &state, 
                               const VectorXd &coeffs) {

  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: 
   * If the state is a 4 element vector, the actuators is a 2 element vector and there are 10 timesteps. 
   * The number of variables is: 4 * 10 + 2 * 9
   */


  // start

  /**
   * Set the number of constraints
   */
  size_t n_vars = 6 * N + 2 * (N - 1);
  size_t n_constraints = 6 * N;

  double t_x    = state[0];
  double t_y    = state[1];
  double t_psi  = state[2];
  double t_v    = state[3];
  double t_cte  = state[4];
  double t_epsi = state[5];

  // end




  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }



  // start

  vars[g_x_start]    = t_x;
  vars[g_y_start]    = t_y;
  vars[g_psi_start]  = t_psi;
  vars[g_v_start]    = t_v;
  vars[g_cte_start]  = t_cte;
  vars[g_epsi_start] = t_epsi;

  // end



  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  /**
   * Set lower and upper limits for variables.
   */


  // start
  
  for (int i = 0; i < g_delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (int i = g_delta_start; i < g_a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (int i = g_a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // end





  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }




  // start

  // Init Upper Bound 
  constraints_upperbound[g_x_start]    = t_x;
  constraints_upperbound[g_y_start]    = t_y;
  constraints_upperbound[g_psi_start]  = t_psi;
  constraints_upperbound[g_v_start]    = t_v;
  constraints_upperbound[g_cte_start]  = t_cte;
  constraints_upperbound[g_epsi_start] = t_epsi;

  // Init Lower Bound 
  constraints_lowerbound[g_x_start]    = t_x;
  constraints_lowerbound[g_y_start]    = t_y;
  constraints_lowerbound[g_psi_start]  = t_psi;
  constraints_lowerbound[g_v_start]    = t_v;
  constraints_lowerbound[g_cte_start]  = t_cte;
  constraints_lowerbound[g_epsi_start] = t_epsi;

  // end




  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * Return the first actuator values. The variables can be accessed with `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0} creates a 2 element double vector.
   */



  // start
  vector<double> predict_result;
  predict_result.push_back(solution.x[g_delta_start]);
  predict_result.push_back(solution.x[g_a_start]);

  for (int i = 0; i < N - 1; i++)
  {
    predict_result.push_back(solution.x[g_x_start + i + 1]);
    predict_result.push_back(solution.x[g_y_start + i + 1]);
  }
  return predict_result;
  // end






}
