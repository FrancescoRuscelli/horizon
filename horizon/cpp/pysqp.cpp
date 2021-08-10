#include "pysqp_helpers.h"

using namespace casadi;

PYBIND11_MODULE(pysqp, m) {

    py::class_<SQPGaussNewton<SX>>(m, "SQPGaussNewton")
            .def(py::init(&constructSX))
            .def("solve", &SQPGaussNewton<SX>::solve)
            .def("getVariableTrajectory", &SQPGaussNewton<SX>::getVariableTrajectory)
            .def("getObjectiveIterations", &SQPGaussNewton<SX>::getObjectiveIterations)
            .def("getConstraintNormIterations", &SQPGaussNewton<SX>::getConstraintNormIterations)
            .def("printConicOptions", &SQPGaussNewton<SX>::printConicOptions)
            ;

    py::class_<SQPGaussNewton<MX>>(m, "SQPGaussNewton")
            .def(py::init(&constructMX))
            .def("solve", &SQPGaussNewton<MX>::solve)
            .def("getVariableTrajectory", &SQPGaussNewton<MX>::getVariableTrajectory)
            .def("getObjectiveIterations", &SQPGaussNewton<MX>::getObjectiveIterations)
            .def("getConstraintNormIterations", &SQPGaussNewton<MX>::getConstraintNormIterations)
            .def("printConicOptions", &SQPGaussNewton<MX>::printConicOptions)
            ;

}
