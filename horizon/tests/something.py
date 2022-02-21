from horizon.problem import Problem
from horizon.solvers import Solver

def test_view():
    nodes = 10
    prb = Problem(nodes)
    x = prb.createStateVariable('x', 5)
    v = prb.createStateVariable('v', 5)
    y = prb.createInputVariable('y', 5)
    z = prb.createVariable('z', 5, [3, 4, 5])
    p = prb.createParameter('p', 4)
    b = prb.createSingleParameter('b', 6)

    x_prev = x.getVarOffset(-2)
    cnsrt1 = prb.createIntermediateConstraint('cnsrt1', x + y)
    cnsrt2 = prb.createConstraint('cnsrt2', x[0:4] + p)
    cnsrt3 = prb.createConstraint('cnsrt3', z + b[1:6], [3, 4, 5])



if __name__ == '__main__':
    # test_singleParameter()
    # test_parameters()
    # test_singleVariable()
    # test_constraintBounds()
    # test_intermediateConstraint()
    # test_variables()
    # test_prev()
    # test_bounds_input()
    # test_bounds_2()
    # test_boundsarray()
    test_view()


#
