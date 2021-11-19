import unittest

from numpy.lib.arraysetops import isin

from horizon.solvers import Solver
from horizon.problem import Problem
from horizon.transcriptions.transcriptor import Transcriptor
import casadi as cs
import numpy as np
np.set_printoptions(suppress=True, precision=3)

class SolverConsistency(unittest.TestCase):
    
    def setUp(self) -> None:
        self.N = 3
        self.prb = Problem(self.N) 
        self.x1 = self.prb.createStateVariable('x1', 2)
        self.x2 = self.prb.createStateVariable('x2', 3)
        self.p1 = self.prb.createParameter('p1', 4)
        self.c = self.prb.createConstraint('c', self.x1)

    def test_bounds(self):
        
        xlb = -np.array([1, 2, 3, 4, 5])
        xub = -xlb 
        xlb_proj = np.repeat(np.atleast_2d(xlb).T, self.N+1, axis=1)
        xub_proj = np.repeat(np.atleast_2d(xub).T, self.N+1, axis=1)
        
        self.prb.getState().setBounds(xlb, xub)
        xlb_output, xub_output = self.prb.getState().getBounds()

        nx = self.prb.getState().getVars().size1()
        self.assertEqual(xlb_output.shape, (nx, self.N+1))
        self.assertEqual(xub_output.shape, (nx, self.N+1))
        self.assertTrue(np.allclose(xlb_proj, xlb_output))
        self.assertTrue(np.allclose(xub_proj, xub_output))

        xlb_output, xub_output = self.x1.getBounds()
        self.assertEqual(xlb_output.shape, (2, self.N+1))
        self.assertEqual(xub_output.shape, (2, self.N+1))
        self.assertTrue(np.allclose(xlb_proj[:2, :], xlb_output))
        self.assertTrue(np.allclose(xub_proj[:2, :], xub_output))

        xlb_output, xub_output = self.x2.getBounds()
        self.assertEqual(xlb_output.shape, (3, self.N+1))
        self.assertEqual(xub_output.shape, (3, self.N+1))
        self.assertTrue(np.allclose(xlb_proj[2:, :], xlb_output))
        self.assertTrue(np.allclose(xub_proj[2:, :], xub_output))

    def test_constr_bounds(self):
        
        glb = -np.array([1, 2])
        gub = -glb 
        glb_proj = np.repeat(np.atleast_2d(glb).T, self.N+1, axis=1)
        gub_proj = np.repeat(np.atleast_2d(gub).T, self.N+1, axis=1)
        
        self.c.setBounds(glb, gub)
        glb_output, gub_output = self.c.getBounds()

        nc = 2
        self.assertEqual(glb_output.shape, (nc, self.N+1))
        self.assertEqual(gub_output.shape, (nc, self.N+1))
        self.assertTrue(np.allclose(glb_proj, glb_output))
        self.assertTrue(np.allclose(gub_proj, gub_output))

    def test_initial_guess(self):
        
        xini = -np.array([1, 2, 3, 4, 5])
        xini_proj = np.repeat(np.atleast_2d(xini).T, self.N+1, axis=1)
        
        self.prb.getState().setInitialGuess(xini)
        xini_output = self.prb.getState().getInitialGuess()

        nx = self.prb.getState().getVars().size1()
        self.assertEqual(xini_output.shape, (nx, self.N+1))
        self.assertTrue(np.allclose(xini_proj, xini_output))

        xini_output = self.x1.getInitialGuess()
        self.assertEqual(xini_output.shape, (2, self.N+1))
        self.assertTrue(np.allclose(xini_proj[0:2, :], xini_output))

        xini_output = self.x2.getInitialGuess()
        self.assertEqual(xini_output.shape, (3, self.N+1))
        self.assertTrue(np.allclose(xini_proj[2:, :], xini_output))


    def test_param_values(self):

        pval = np.array([1, 2, 3, 4])
        pval_proj = np.repeat(np.atleast_2d(pval).T, self.N+1, axis=1)

        self.p1.assign(pval)

        pval_output = self.p1.getValues()

        self.assertEqual(type(pval_output), np.ndarray)

        self.assertTrue(pval_output.shape, (4, self.N+1))
        self.assertTrue(np.allclose(pval_proj, pval_output))

if __name__ == '__main__':
    unittest.main()