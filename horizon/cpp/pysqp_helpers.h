#ifndef PYSQP_HELPERS_H
#define PYSQP_HELPERS_H

#include "src/sqp.h"
#include "pyilqr_helpers.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <typeinfo>

namespace py = pybind11;
using namespace horizon;














template <typename T>
bool setOption(const std::string& key, const std::string& solver_key, py::handle& value, casadi::Dict& dict)
{
    bool success = false;
    if(key == solver_key)
    {
        dict[key] = value.cast<T>();
        success = true;
    }
    return success;
}


bool checkOptions(const std::string& key, py::handle& value, casadi::Dict& dict)
{
    // -- sqp options --//
    if(setOption    <int>           (key,   "max_iter",                         value, dict)) return true;
    if(setOption    <bool>          (key,   "reinitialize_qpsolver",            value, dict)) return true;
    if(setOption    <double>        (key,   "solution_convergence",             value, dict)) return true;
    // -- qpoases options --//
    if(setOption    <bool>          (key,   "sparse",                           value, dict)) return true;
    if(setOption    <bool>          (key,   "schur",                            value, dict)) return true;
    if(setOption    <std::string>   (key,   "hessian_type",                     value, dict)) return true;
    if(setOption    <int>           (key,   "max_schur",                        value, dict)) return true;
    if(setOption    <std::string>   (key,   "linsol_plugin",                    value, dict)) return true;
    if(setOption    <int>           (key,   "nWSR",                             value, dict)) return true;
    if(setOption    <double>        (key,   "CPUtime",                          value, dict)) return true;
    if(setOption    <std::string>   (key,   "printLevel",                       value, dict)) return true;
    if(setOption    <bool>          (key,   "enableRamping",                    value, dict)) return true;
    if(setOption    <bool>          (key,   "enableFarBounds",                  value, dict)) return true;
    if(setOption    <bool>          (key,   "enableFlippingBounds",             value, dict)) return true;
    if(setOption    <bool>          (key,   "enableRegularisation",             value, dict)) return true;
    if(setOption    <bool>          (key,   "enableFullLITests",                value, dict)) return true;
    if(setOption    <bool>          (key,   "enableNZCTests",                   value, dict)) return true;
    if(setOption    <int>           (key,   "enableDriftCorrection",            value, dict)) return true;
    if(setOption    <int>           (key,   "enableCholeskyRefactorisation",    value, dict)) return true;
    if(setOption    <bool>          (key,   "enableEqualities",                 value, dict)) return true;
    if(setOption    <double>        (key,   "terminationTolerance",             value, dict)) return true;
    if(setOption    <double>        (key,   "boundTolerance",                   value, dict)) return true;
    if(setOption    <double>        (key,   "boundRelaxation",                  value, dict)) return true;
    if(setOption    <double>        (key,   "epsNum",                           value, dict)) return true;
    if(setOption    <double>        (key,   "epsDen",                           value, dict)) return true;
    if(setOption    <double>        (key,   "maxPrimalJump",                    value, dict)) return true;
    if(setOption    <double>        (key,   "maxDualJump",                      value, dict)) return true;
    if(setOption    <double>        (key,   "initialRamping",                   value, dict)) return true;
    if(setOption    <double>        (key,   "finalRamping",                     value, dict)) return true;
    if(setOption    <double>        (key,   "initialFarBounds",                 value, dict)) return true;
    if(setOption    <double>        (key,   "growFarBounds",                    value, dict)) return true;
    if(setOption    <std::string>   (key,   "initialStatusBounds",              value, dict)) return true;
    if(setOption    <double>        (key,   "epsFlipping",                      value, dict)) return true;
    if(setOption    <int>           (key,   "numRegularisationSteps",           value, dict)) return true;
    if(setOption    <double>        (key,   "epsRegularisation",                value, dict)) return true;
    if(setOption    <int>           (key,   "numRefinementSteps",               value, dict)) return true;
    if(setOption    <double>        (key,   "epsIterRef",                       value, dict)) return true;
    if(setOption    <double>        (key,   "epsLITests",                       value, dict)) return true;
    if(setOption    <double>        (key,   "epsNZCTests",                      value, dict)) return true;
    if(setOption    <bool>          (key,   "enableInertiaCorrection",          value, dict)) return true;

    return false;


}




auto constructMX(const std::string& name, const std::string& qp_solver,
               py::object f, py::object g,
               const py::dict& opt = py::dict())
{
    casadi::Dict casadi_opts = casadi::Dict();

    for (std::pair<py::handle, py::handle> item : opt)
    {
        if(!checkOptions(item.first.cast<std::string>(), item.second, casadi_opts))
            std::cout<<"option: "<<item.first.cast<std::string>()<<"missing from parser!"<<std::endl;
    }

    return std::make_unique< SQPGaussNewton< casadi::MX > >(name, qp_solver, to_cpp(f), to_cpp(g), casadi_opts);
}

auto constructSX(std::string name, std::string qp_solver,
               py::object f, py::object g,
               const py::dict& opt = py::dict())
{
    casadi::Dict casadi_opts = casadi::Dict();

    for (std::pair<py::handle, py::handle> item : opt)
    {
        if(!checkOptions(item.first.cast<std::string>(), item.second, casadi_opts))
            std::cout<<"option: "<<item.first.cast<std::string>()<<"missing from parser!"<<std::endl;
    }

    return std::make_unique< SQPGaussNewton< casadi::SX > >(name, qp_solver, to_cpp(f), to_cpp(g), casadi_opts);
}

bool fMX(SQPGaussNewton<casadi::MX>& self, py::object f, bool reinitialize_qp_solver = true)
{
    return self.f(to_cpp(f), reinitialize_qp_solver);
}

bool fSX(SQPGaussNewton<casadi::SX>& self, py::object f, bool reinitialize_qp_solver = true)
{
    return self.f(to_cpp(f), reinitialize_qp_solver);
}

auto callMX(SQPGaussNewton<casadi::MX>& self, const Eigen::VectorXd& x0,
                            const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx,
                            const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg,
                            const Eigen::MatrixXd& p = Eigen::MatrixXd())
{
    casadi::DM _x0_, _lbx_, _ubx_, _lbg_, _ubg_, _p_;
    casadi_utils::toCasadiMatrix(x0, _x0_);
    casadi_utils::toCasadiMatrix(lbx, _lbx_);
    casadi_utils::toCasadiMatrix(ubx, _ubx_);
    casadi_utils::toCasadiMatrix(lbg, _lbg_);
    casadi_utils::toCasadiMatrix(ubg, _ubg_);
    casadi_utils::toCasadiMatrix(p, _p_);
    casadi::DMDict tmp = self.solve(_x0_, _lbx_, _ubx_, _lbg_, _ubg_, _p_);

    py::dict solution;
    Eigen::VectorXd x;
    casadi_utils::toEigen(tmp.at("x"), x);
    solution["x"] = x;
    solution["f"] = double(tmp.at("f")(0));
    solution["g"] = double(tmp.at("g")(0));
    return solution;
}


auto callSX(SQPGaussNewton<casadi::SX>& self, const Eigen::VectorXd& x0,
                            const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx,
                            const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg,
                            const Eigen::MatrixXd& p = Eigen:: MatrixXd())
{
    casadi::DM _x0_, _lbx_, _ubx_, _lbg_, _ubg_, _p_;
    casadi_utils::toCasadiMatrix(x0, _x0_);
    casadi_utils::toCasadiMatrix(lbx, _lbx_);
    casadi_utils::toCasadiMatrix(ubx, _ubx_);
    casadi_utils::toCasadiMatrix(lbg, _lbg_);
    casadi_utils::toCasadiMatrix(ubg, _ubg_);
    casadi_utils::toCasadiMatrix(p, _p_);
    casadi::DMDict tmp = self.solve(_x0_, _lbx_, _ubx_, _lbg_, _ubg_, _p_);

    py::dict solution;
    Eigen::VectorXd x;
    casadi_utils::toEigen(tmp.at("x"), x);
    solution["x"] = x;
    solution["f"] = double(tmp.at("f")(0));
    solution["g"] = double(tmp.at("g")(0));
    return solution;
}

#endif
