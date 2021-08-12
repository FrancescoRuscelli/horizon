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



auto constructMX(const std::string& name, const std::string& qp_solver,
               py::object f, py::object g,
               const py::dict& opt = py::dict())
{
    casadi::Dict casadi_opts = casadi::Dict();

    for (std::pair<py::handle, py::handle> item : opt)
    {
        auto key = item.first.cast<std::string>();
        casadi_opts[key] = item.second.cast<casadi::GenericType>();

    }

    return std::make_unique< SQPGaussNewton< casadi::MX > >(name, qp_solver, to_cpp(f), to_cpp(g), casadi_opts);
}

auto constructSX(std::string name, std::string qp_solver,
               py::object f, py::object g,
               const casadi::Dict& opt = casadi::Dict())
{


    return std::make_unique< SQPGaussNewton< casadi::SX > >(name, qp_solver, to_cpp(f), to_cpp(g), opt);
}

auto callMX(SQPGaussNewton<casadi::MX>& self, const Eigen::VectorXd& x0,
                            const Eigen::VectorXd& lbx, const Eigen::VectorXd& ubx,
                            const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg)
{
    casadi::DM _x0_, _lbx_, _ubx_, _lbg_, _ubg_;
    casadi_utils::toCasadiMatrix(x0, _x0_);
    casadi_utils::toCasadiMatrix(lbx, _lbx_);
    casadi_utils::toCasadiMatrix(ubx, _ubx_);
    casadi_utils::toCasadiMatrix(lbg, _lbg_);
    casadi_utils::toCasadiMatrix(ubg, _ubg_);
    casadi::DMDict tmp = self.solve(_x0_, _lbx_, _ubx_, _lbg_, _ubg_);

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
                            const Eigen::VectorXd& lbg, const Eigen::VectorXd& ubg)
{
    casadi::DM _x0_, _lbx_, _ubx_, _lbg_, _ubg_;
    casadi_utils::toCasadiMatrix(x0, _x0_);
    casadi_utils::toCasadiMatrix(lbx, _lbx_);
    casadi_utils::toCasadiMatrix(ubx, _ubx_);
    casadi_utils::toCasadiMatrix(lbg, _lbg_);
    casadi_utils::toCasadiMatrix(ubg, _ubg_);
    casadi::DMDict tmp = self.solve(_x0_, _lbx_, _ubx_, _lbg_, _ubg_);

    py::dict solution;
    Eigen::VectorXd x;
    casadi_utils::toEigen(tmp.at("x"), x);
    solution["x"] = x;
    solution["f"] = double(tmp.at("f")(0));
    solution["g"] = double(tmp.at("g")(0));
    return solution;
}

#endif
