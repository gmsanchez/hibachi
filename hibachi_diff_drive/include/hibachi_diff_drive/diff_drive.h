#ifndef HIBACHI_DIFFDRIVE_DIFF_DRIVE_H
#define HIBACHI_DIFFDRIVE_DIFF_DRIVE_H

#include <casadi/casadi.hpp>
// #include <string>
// #include <chrono>
#include <vector>
#include <limits>

/**
 * This example configures an optimization problem
 * to solve the differential drive kinematics
 * with restrictions.

 \author Guido Sanchez
 \date 2022
*/

// using namespace casadi;
// using namespace std;

namespace hibachi_diff_drive
{

class DifferentialDrive
{
public:
    // DifferentialDrive(double wheel_radius, double wheel_base);
    // DifferentialDrive(double wheel_radius, double wheel_base, double v_body_max, double w_body_max);
    DifferentialDrive(double wheel_radius = 0.05,
                      double wheel_base = 1.0,
                      double v_body_max = std::numeric_limits<double>::infinity(),
                      double w_body_max = std::numeric_limits<double>::infinity(),
                      double w_left_max = std::numeric_limits<double>::infinity(),
                      double w_right_max = std::numeric_limits<double>::infinity()) : wheel_radius_(wheel_radius),
                                                                                 wheel_base_(wheel_base),
                                                                                 v_body_max_(v_body_max),
                                                                                 w_body_max_(w_body_max),
                                                                                 w_left_max_(w_left_max),
                                                                                 w_right_max_(w_right_max),
                                                                                 v_body_ref_(0.0),
                                                                                 w_body_ref_(0.0)
    {
        this->CreateSolver();
    };

    std::vector<double> solve(double v_body_ref, double w_body_ref)
    {
        double * v = &this->v_body_ref_;

        this->v_body_ref_ = v_body_ref;
        this->w_body_ref_ = w_body_ref;
        this->arg_["p"] = {this->v_body_ref_, this->w_body_ref_};

        this->res_ = this->solver_(this->arg_);

        std::vector<double> uopt(this->res_.at("x"));
        // std::vector<double> uopt;

        std::cout << "----- from within the class -----" << std::endl;
        // cout << "objective at solution = " << this->res_.at("f") << endl;
        std::cout << "primal solution = " << this->res_.at("x") << std::endl;
        // v_sol_body = uopt[0];
        // w_sol_body = uopt[1];
        return uopt;
    };

private:
    void CreateSolver(){
        // Optimization variables
        casadi::SX v_body = casadi::SX::sym("v_body", 1);
        casadi::SX w_body = casadi::SX::sym("w_body", 1);
        casadi::SX w_left_wheel = casadi::SX::sym("w_left_wheel", 1);
        casadi::SX w_right_wheel = casadi::SX::sym("w_right_wheel", 1);

        // Optimization parameters
        casadi::SX v_body_ref = casadi::SX::sym("v_body_ref", 1);
        casadi::SX w_body_ref = casadi::SX::sym("w_body_ref", 1);

        // Optimization variables and parameters as arrays
        this->opt_var_ = casadi::SX::vertcat({v_body, w_body, w_left_wheel, w_right_wheel});
        this->opt_par_ = casadi::SX::vertcat({v_body_ref, w_body_ref});

        // Objective function
        casadi::SX f = casadi::SX::pow(this->opt_var_(0) - this->opt_par_(0), 2.0) + casadi::SX::pow(this->opt_var_(1) - this->opt_par_(1), 2.0);

        // Constraints
        std::vector<casadi::SX> g_k = {this->opt_var_(0) - (this->wheel_radius_ / 2.0) * (this->opt_var_(2) + this->opt_var_(3)),
                          this->opt_var_(1) - (this->wheel_radius_ / this->wheel_base_) * (-this->opt_var_(2) + this->opt_var_(3))};

        casadi::SX g = casadi::SX::vertcat(g_k);
        std::cout << "restricciones: " << g << std::endl;
        // Pick an NLP solver
        // std::string MySolver = "ipopt";
        // std::string MySolver = "worhp";
        std::string MySolver = "sqpmethod";
        // std::string MySolver = "sqpmethod_ipopt";

        // Set options
        casadi::Dict opts;

        if (MySolver == "sqpmethod")
        {
            opts["qpsol"] = "qpoases";
            opts["qpsol_options.printLevel"] = "none";
            opts["print_time"] = 0;
        }
        if (MySolver == "sqpmethod_ipopt")
        {
            MySolver = "sqpmethod";
            opts["expand"] = true;
            // opts["max_iter"] = 10)
            // opts["verbose"] = true;
            // opts["linear_solver"] = "ma57";
            opts["hessian_approximation"] = "exact";
            // opts["derivative_test"] = "second-order";

            // Specify QP solver
            opts["qpsol"] = "nlpsol";
            opts["qpsol_options.nlpsol"] = "ipopt";
            opts["qpsol_options.error_on_fail"] = false;
            opts["qpsol_options.nlpsol_options.ipopt.print_level"] = 0;
            opts["qpsol_options.nlpsol_options.print_time"] = 0;
            opts["qpsol_options.nlpsol_options.ipopt.sb"] = "yes";
        }

        if (MySolver == "ipopt")
        {
            opts["ipopt.hessian_approximation"] = "exact";
            // opts["ipopt.tol"] = 1e-5;
            opts["ipopt.sb"] = "yes";
            opts["ipopt.print_level"] = 0;
            opts["print_time"] = 0;
            opts["ipopt.max_iter"] = 100;
        }

        // Create an NLP solver instance
        this->solver_ = casadi::nlpsol("solver", MySolver, (casadi::SXDict){{"x", this->opt_var_}, {"f", f}, {"g", g}, {"p", this->opt_par_}}, opts);

        // Bounds and initial guess
        // std::map<std::string, DM> arg, res;
        this->arg_["lbx"] = {-this->v_body_max_, -this->w_body_max_, -this->w_left_max_, -this->w_right_max_};
        this->arg_["ubx"] = {+this->v_body_max_, +this->w_body_max_, +this->w_left_max_, +this->w_right_max_};
        this->arg_["lbg"] = {0.0, 0.0};
        this->arg_["ubg"] = {0.0, 0.0};
        this->arg_["x0"] = {0.0, 0.0, 0.0, 0.0};
        this->arg_["p"] = {this->v_body_ref_, this->w_body_ref_};
        // this->v_body_max_ = arg_["p"]->at(0);
        // this->w_body_max_ = arg_["p"]->at(1);
    };

    double wheel_radius_;
    double wheel_base_;
    double v_body_max_;
    double w_body_max_;
    double w_left_max_;
    double w_right_max_;
    double v_body_ref_;
    double w_body_ref_;
    casadi::SX opt_var_; // optimization problem variables
    casadi::SX opt_par_; // optimization problem parameters
    std::map<std::string, casadi::DM> arg_, res_;
    casadi::Function solver_;
};

}// namespace hibachi_diff_drive

#endif // !HIBACHI_DIFFDRIVE_DIFF_DRIVE_H