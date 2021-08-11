#include "pysqp_helpers.h"
#include <casadi/casadi.hpp>


PYBIND11_MODULE(pysqp, m) {

    py::class_<SQPGaussNewton<casadi::SX>>(m, "SQPGaussNewtonSX")
            .def(py::init(&constructSX))
            .def("solve", callSX,
                 py::arg("x0"), py::arg("lbx"), py::arg("ubx"), py::arg("lbg"), py::arg("ubg"))
            .def("setAlpha", &SQPGaussNewton<casadi::SX>::setAlpha)
            .def("getAlpha", &SQPGaussNewton<casadi::SX>::getAlpha)
            .def("getVariableTrajectory", &SQPGaussNewton<casadi::SX>::getVariableTrajectory)
            .def("getObjectiveIterations", &SQPGaussNewton<casadi::SX>::getObjectiveIterations)
            .def("getConstraintNormIterations", &SQPGaussNewton<casadi::SX>::getConstraintNormIterations)
            .def("printConicOptions", &SQPGaussNewton<casadi::SX>::printConicOptions)
            ;

//    py::class_<SQPGaussNewton<casadi::MX>>(m, "SQPGaussNewtonMX")
//            .def(py::init(&constructMX))
//            .def("solve", &SQPGaussNewton<casadi::MX>::solve,
//                 py::arg("x0"), py::arg("lbx"), py::arg("ubx"), py::arg("lbg"), py::arg("ubg"))
//            .def("setAlpha", &SQPGaussNewton<casadi::MX>::setAlpha)
//            .def("getAlpha", &SQPGaussNewton<casadi::MX>::getAlpha)
//            .def("getVariableTrajectory", &SQPGaussNewton<casadi::MX>::getVariableTrajectory)
//            .def("getObjectiveIterations", &SQPGaussNewton<casadi::MX>::getObjectiveIterations)
//            .def("getConstraintNormIterations", &SQPGaussNewton<casadi::MX>::getConstraintNormIterations)
//            .def("printConicOptions", &SQPGaussNewton<casadi::MX>::printConicOptions)
//            ;

}
