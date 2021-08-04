#include "pyilqr_helpers.h"

PYBIND11_MODULE(pyilqr, m) {

    py::class_<IterativeLQR>(m, "IterativeLQR")
            .def(py::init(&construct))
            .def("setIntermediateCost", set_inter_cost_wrapper)
            .def("setIntermediateCost", set_inter_cost_wrapper_single)
            .def("setIntermediateConstraint", set_inter_constraint_wrapper)
            .def("setIntermediateConstraint", set_inter_constraint_wrapper_single)
            .def("setFinalCost", set_final_cost_wrapper)
            .def("setFinalConstraint", set_final_constraint_wrapper)
            .def("solve", &IterativeLQR::solve)
            .def("backward_pass", &IterativeLQR::backward_pass)
            .def("forward_pass", &IterativeLQR::forward_pass)
            .def("linearize_quadratize", &IterativeLQR::linearize_quadratize)
            .def("getStateTrajectory", &IterativeLQR::getStateTrajectory)
            .def("getInputTrajectory", &IterativeLQR::getInputTrajectory)
            .def("setInitialState", &IterativeLQR::setInitialState)
            ;

}



