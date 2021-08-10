#ifndef PYILQR_HELPERS_H
#define PYILQR_HELPERS_H

#include "src/sqp.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;
using namespace horizon;

auto constructMX(const std::string& name, const std::string& qp_solver,
               const casadi::MX& f, const casadi::MX& g, const casadi::MX& x,
               const casadi::Dict& opt = casadi::Dict())
{
    return std::make_unique< SQPGaussNewton< casadi::MX > >(name, qp_solver, f, g, x, opt);
}

auto constructSX(const std::string& name, const std::string& qp_solver,
               const casadi::SX& f, const casadi::SX& g, const casadi::SX& x,
               const casadi::Dict& opt = casadi::Dict())
{
    return std::make_unique< SQPGaussNewton< casadi::SX > >(name, qp_solver, f, g, x, opt);
}

#endif
