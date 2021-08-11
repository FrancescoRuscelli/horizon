#include "pyilqr_helpers.h"

PYBIND11_MODULE(pyilqr, m) {

    py::class_<IterativeLQR::ForwardPassResult>(m, "ForwardPassResult")
            .def_readonly("xtrj", &IterativeLQR::ForwardPassResult::xtrj)
            .def_readonly("utrj", &IterativeLQR::ForwardPassResult::utrj)
            .def_readonly("accepted", &IterativeLQR::ForwardPassResult::accepted)
            .def_readonly("alpha", &IterativeLQR::ForwardPassResult::alpha)
            .def_readonly("constraint_violation", &IterativeLQR::ForwardPassResult::constraint_violation)
            .def_readonly("cost", &IterativeLQR::ForwardPassResult::cost)
            .def_readonly("defect_norm", &IterativeLQR::ForwardPassResult::defect_norm)
            .def_readonly("merit", &IterativeLQR::ForwardPassResult::merit)
            .def_readonly("step_length", &IterativeLQR::ForwardPassResult::step_length);

    py::class_<utils::ProfilingInfo>(m, "ProfilingInfo")
            .def_readonly("timings", &utils::ProfilingInfo::timings);

    py::class_<IterativeLQR>(m, "IterativeLQR")
            .def(py::init(&construct))
            .def("setIntermediateCost", set_inter_cost_wrapper)
            .def("setIntermediateCost", set_inter_cost_wrapper_single)
            .def("setIntermediateConstraint", set_inter_constraint_wrapper)
            .def("setIntermediateConstraint", set_inter_constraint_wrapper_single)
            .def("setFinalCost", set_final_cost_wrapper)
            .def("setFinalConstraint", set_final_constraint_wrapper)
            .def("setStepLength", &IterativeLQR::setStepLength)
            .def("solve", &IterativeLQR::solve)
            .def("getProfilingInfo", &IterativeLQR::getProfilingInfo)
            .def("setIterationCallback", &IterativeLQR::setIterationCallback)
            .def("getStateTrajectory", &IterativeLQR::getStateTrajectory)
            .def("getInputTrajectory", &IterativeLQR::getInputTrajectory)
            .def("setInitialState", &IterativeLQR::setInitialState)
            .def("setInputInitialGuess", &IterativeLQR::setInputInitialGuess)
            .def("setStateInitialGuess", &IterativeLQR::setStateInitialGuess)
            ;

}



