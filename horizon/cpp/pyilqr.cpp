#include "src/ilqr.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace horizon;

casadi::Function to_cpp(py::object pyfn)
{
    // convert python's casadi.Function to cpp's casadi::Function
    auto cs = py::module_::import("casadi");
    auto Function = cs.attr("Function");
    auto serialize = Function.attr("serialize");
    auto fstr = serialize(pyfn).cast<std::string>();

    return casadi::Function::deserialize(fstr);
}

auto construct(py::object fdyn, int N)
{
    return std::make_unique<IterativeLQR>(to_cpp(fdyn), N);
}

auto set_inter_cost_wrapper(IterativeLQR& self, std::vector<py::object> flist)
{
    std::vector<casadi::Function> flist_cpp;
    for(auto pyfn : flist)
    {
        flist_cpp.push_back(to_cpp(pyfn));
    }

    self.setIntermediateCost(flist_cpp);

}

auto set_final_cost_wrapper(IterativeLQR& self, py::object pyfn)
{
    self.setFinalCost(to_cpp(pyfn));
}

auto set_final_constraint_wrapper(IterativeLQR& self, py::object pyfn)
{
    self.setFinalConstraint(to_cpp(pyfn));
}



PYBIND11_MODULE(pyilqr, m) {

    py::class_<IterativeLQR>(m, "IterativeLQR")
            .def(py::init(&construct))
            .def("setIntermediateCost", set_inter_cost_wrapper)
            .def("setFinalCost", set_final_cost_wrapper)
            .def("setFinalConstraint", set_final_constraint_wrapper)
            .def("backward_pass", &IterativeLQR::backward_pass)
            .def("forward_pass", &IterativeLQR::forward_pass)
            .def("linearize_quadratize", &IterativeLQR::linearize_quadratize)
            .def("getStateTrajectory", &IterativeLQR::getStateTrajectory)
            .def("linearize_quadratize", &IterativeLQR::getInputTrajectory)
            .def("setInitialState", &IterativeLQR::setInitialState)
            ;


}



