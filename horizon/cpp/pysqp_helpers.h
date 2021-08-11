#ifndef PYSQP_HELPERS_H
#define PYSQP_HELPERS_H

#include "src/sqp.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;
using namespace horizon;

/// TODO: THIS METHOD IF COMPILED GENERATES undefined symbol: _ZN6casadi18UniversalNodeOwnerD1Ev
/// IN PYTHON BINDINGS!
//casadi::MX to_cppMX(py::object casadi_obj)
//{
//    // convert python's casadi.Function to cpp's casadi::Function
//#if PYBIND11_VERSION_MINOR > 6
//    auto cs = py::module_::import("casadi");
//#else
//    auto cs = py::module::import("casadi");
//#endif
//    std::string casadi_type_str = "MX";

//    auto tmp = cs.attr(casadi_type_str.c_str());
//    auto serialize = tmp.attr("serialize");

//    auto str = serialize(casadi_obj).cast<std::string>();
//    std::istringstream istr(str);
//    casadi::DeserializingStream dstr(istr);

//    return casadi::MX::deserialize(dstr); /// <---- DUE TO THIS!!!!
//}

casadi::SX to_cppSX(py::object casadi_obj)
{
    // convert python's casadi.Function to cpp's casadi::Function
#if PYBIND11_VERSION_MINOR > 6
    auto cs = py::module_::import("casadi");
#else
    auto cs = py::module::import("casadi");
#endif
    std::string casadi_type_str = "SX";

    auto tmp = cs.attr(casadi_type_str.c_str());
    auto serialize = tmp.attr("serialize");

    auto istr = serialize(casadi_obj).cast<std::string>();

    return casadi::SX::deserialize(istr);
}

//auto constructMX(const std::string& name, const std::string& qp_solver,
//               py::object f, py::object g, py::object x,
//               const casadi::Dict& opt = casadi::Dict())
//{
//    return std::make_unique< SQPGaussNewton< casadi::MX > >(name, qp_solver,
//                            to_cppMX(f), to_cppMX(g), to_cppMX(x), opt);
//}

auto constructSX(std::string name, std::string qp_solver,
               py::object f, py::object g, py::object x,
               casadi::Dict opt = casadi::Dict())
{
    return std::make_unique< SQPGaussNewton< casadi::SX > >(name, qp_solver,
                            to_cppSX(f), to_cppSX(g), to_cppSX(x), opt);
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
    return self.solve(_x0_, _lbx_, _ubx_, _lbg_, _ubg_);
}

#endif
