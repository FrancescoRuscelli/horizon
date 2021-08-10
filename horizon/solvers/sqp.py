import casadi as cs
import time
import numpy
from horizon.utils import utils


def sqpsol(name, qp_solver, problem_dict, options_dict):
    """
    sqpsol creates a sqp solver using Gauss-Newton approximation of the Hessian
    Args:
        name: name of the solver (not used at the moment)
        qp_solver: internal qp solver name
        problem_dict: {'f': cost_function (residual!), 'g': constraints, 'x': variables}
        options_dict: {'max_iter': iterations}

    Returns: a sqp object

    NOTE: the cost function has to be passed in a residual form: [f1, f2, f3, ...]
    where F = ||f1|| + ||f2|| + ...

    """
    return sqp(name, qp_solver, problem_dict, options_dict)

def qpoasesMPCOptions():
    opts = {'qpoases.sparse': True,
            'qpoases.linsol_plugin': 'ma57',
            'qpoases.enableRamping': False,
            'qpoases.enableFarBounds': False,
            'qpoases.enableFlippingBounds': False,
            'qpoases.enableFullLITests': False,
            'qpoases.enableNZCTests': False,
            'qpoases.enableDriftCorrection': 0,
            'qpoases.enableCholeskyRefactorisation': 0,
            'qpoases.enableEqualities': True,
            'qpoases.initialStatusBounds': 'inactive',
            'qpoases.numRefinementSteps': 0,
            'qpoases.terminationTolerance': 1e9 * np.finfo(float).eps,
            'qpoases.enableInertiaCorrection': False,
            'qpoases.printLevel': 'none'}
    return opts

def osqpOptions():
    opts = {'osqp.osqp': {'verbose': False}}

    return opts

class sqp(object):
    """
    Implements a sqp solver using Gauss-Newton approximation of the Hessian
    """
    def __init__(self, name, qp_solver, problem_dict, options_dict):
        """

        Args:
            name: name of the solver (not used at the moment)
            problem_dict: {'f': residual of cost function, 'g': constraints, 'x': variables}
            options_dict: {'max_iter': iterations, 'qpsolver': internal_qpsolver
        """
        self.__name = name
        self.__problem_dict = problem_dict
        self.__options_dict = options_dict
        self.__qpsolver = qp_solver

        self.__qpsolver_options = self.qpsolver_option_parser(self.__qpsolver, self.__options_dict)

        self.__f = self.__problem_dict['f']
        self.__g = None
        if 'g' in self.__problem_dict:
            self.__g = self.__problem_dict['g']

        self.__x = self.__problem_dict['x']

        self.__max_iter = 1000
        if 'max_iter' in self.__options_dict:
            self.__max_iter = self.__options_dict['max_iter']

        self.__reinitialize_qpsolver = False
        if 'reinitialize_qpsolver' in self.__options_dict:
            self.__reinitialize_qpsolver = self.__options_dict['reinitialize_qpsolver']




        # Form function for calculating the Gauss-Newton objective
        self.__r_fcn = cs.Function('r_fcn', {'v': self.__x, 'r': self.__f}, ['v'], ['r'])

        # Form function for calculating the constraints
        self.__g_fcn = []
        if self.__g is not None:
            self.__g_fcn = cs.Function('g_fcn', {'v': self.__x, 'g': self.__g}, ['v'], ['g'])

        # Generate functions for the Jacobians
        # self.__Jac_r_fcn = self.__r_fcn.jac()
        self.__Jac_r_fcn, _ = utils.jac({'v': self.__x, 'r': self.__f}, ['v'], ['r'])

        self.__Jac_g_fcn = []
        if self.__g is not None:
            # self.__Jac_g_fcn = self.__g_fcn.jac()
            self.__Jac_g_fcn, _ = utils.jac({'v': self.__x, 'g': self.__g}, ['v'], ['g'])

        self.__v0 = []
        self.__vmin = []
        self.__vmax = []
        self.__gmin = []
        self.__gmax = []

        self.__v_opt = []
        self.__obj = []
        self.__constr = []

        self.__solver = []
        self.__qp = {}

    def qpsolve(self, H, g, lbx, ubx, A, lba, uba, init=True):
        """
        Internal qp solver to solve differential problem
        Args:
            H: Hessian cos function
            g: gradient cost function
            lbx: lower bounds
            ubx: upper bounds
            A: Constraints
            lba: lower constraints
            uba: upper constraints

        Returns: solution of differential problem

        """

        if init:
            # QP structure
            self.__qp['h'] = H.sparsity()
            if A is not None:
                self.__qp['a'] = A.sparsity()

            # Create CasADi solver instance
            self.__solver = cs.conic('S', self.__qpsolver, self.__qp, self.__qpsolver_options)

        if A is None:
            A = []
        r = self.__solver(h=H, g=g, a=A, lbx=lbx, ubx=ubx, lba=lba, uba=uba)

        # Return the solution
        return r['x']

    def __call__(self, x0, lbx, ubx, lbg=None, ubg=None, p=None, alpha=1.):
        """
        Compute solution of non linear problem
        Args:
            x0: initial guess
            lbx: lower bounds
            ubx: upper bounds
            lbg: lower constraints
            ubg: upper constraints
            p: parameters NOTE USED AT THE MOMENT!
            alpha: step

        Returns: solution dict {'x': nlp_solution, 'f': value_cost_function, 'g': value_constraints}

        """

        if ubg is None:
            ubg = []
        if lbg is None:
            lbg = []
        self.__hessian_computation_time = []
        self.__qp_computation_time = []

        self.__v0 = x0
        self.__vmin = lbx
        self.__vmax = ubx
        self.__gmin = lbg
        self.__gmax = ubg

        self.__v_opt = self.__v0
        for k in range(self.__max_iter):

            init = self.__reinitialize_qpsolver
            if k == 0:
                init = True

            # Form quadratic approximation of objective
            Jac_r_fcn_value = self.__Jac_r_fcn(v=self.__v_opt)  # evaluate in v_opt
            J_r_k = Jac_r_fcn_value['DrDv']
            r_k = self.__r_fcn(v=self.__v_opt)['r']

            # Form quadratic approximation of constraints
            Jac_g_fcn_value = []
            J_g_k = None
            g_k = []
            if self.__g is not None:
                Jac_g_fcn_value = self.__Jac_g_fcn(v=self.__v_opt)  # evaluate in v_opt
                J_g_k = Jac_g_fcn_value['DgDv']
                g_k = self.__g_fcn(v=self.__v_opt)['g']

            # Gauss-Newton Hessian
            t = time.time()
            H_k = cs.mtimes(J_r_k.T, J_r_k)
            elapsed = time.time() - t
            self.__hessian_computation_time.append(elapsed)

            # Gradient of the objective function
            Grad_obj_k = cs.mtimes(J_r_k.T, r_k)

            # Bounds on delta_v
            dv_min = self.__vmin - self.__v_opt
            dv_max = self.__vmax - self.__v_opt

            # Solve the QP
            t = time.time()
            dv = self.qpsolve(H_k, Grad_obj_k, dv_min, dv_max, J_g_k, -g_k + self.__gmin, -g_k + self.__gmax, init)
            elapsed = time.time() - t
            self.__qp_computation_time.append(elapsed)

            # Take the full step
            self.__v_opt += alpha*dv.toarray().flatten()
            self.__obj.append(float(numpy.dot(r_k.T, r_k) / 2.))
            if self.__g is not None:
                self.__constr.append(float(cs.norm_2(g_k)))

        solution_dict = {'x': cs.DM(self.__v_opt.tolist()), 'f': self.__obj, 'g': self.__constr}
        return solution_dict

    def f(self, f):
        """
        Permits to update cost function
        Args:
            f: new cost function
        """
        self.__f = f
        # Form function for calculating the Gauss-Newton objective
        self.__r_fcn = cs.Function('r_fcn', {'v': self.__x, 'r': self.__f}, ['v'], ['r'])
        # Generate functions for the Jacobians
        # self.__Jac_r_fcn = self.__r_fcn.jac()
        self.__Jac_r_fcn, _ = utils.jac({'v': self.__x, 'r': self.__f}, ['v'], ['r'])

    def g(self, g):
        """
        permits to update constraints
        Args:
            g: new constraints
        """
        self.__g = g
        self.__g_fcn = cs.Function('g_fcn', {'v': self.__x, 'g': self.__g}, ['v'], ['g'])
        # self.__Jac_g_fcn = self.__g_fcn.jac()
        self.__Jac_g_fcn, _ = utils.jac({'v': self.__x, 'g': self.__g}, ['v'], ['g'])

    def qpsolver_option_parser(self, qpsolver, options):
        parsed_options = {}
        for key in options:
            list = key.split(".")
            if list[0] == qpsolver:
                parsed_options[list[1]] = options[key]
        return parsed_options


    def plot(self):
        """
        Shows plots of value of objective function and constraint violation
        """
        import matplotlib.pyplot as plt
        # Plot the results
        plt.figure(1)

        plt.title("SQP solver output")
        plt.semilogy(self.__obj)
        if self.__constr:
            plt.semilogy(self.__constr)
        plt.xlabel('iteration')
        if self.__constr:
            plt.legend(['Objective value', 'Constraint violation'], loc='center right')
        else:
            plt.legend(['Objective value'], loc='center right')
        plt.grid()

        plt.show()

    def get_qp_computation_time(self):
        """

        Returns: list of logged time to setup and solve a QP

        """
        return self.__qp_computation_time

    def get_hessian_computation_time(self):
        """

        Returns: list of logged time to compute the hessian

        """
        return self.__hessian_computation_time
