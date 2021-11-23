from .nlpsol import NlpsolSolver
from horizon.problem import Problem
from typing import Dict
import casadi as cs

class BlockSqpSolver(NlpsolSolver):

    def __init__(self, prb: Problem, opts: Dict) -> None:
        
        super().__init__(prb, opts=opts, solver_plugin='blocksqp')

    def configure_rti(self) -> bool:
        rti_opts = {
                    'warmstart': False,
                    'max_iter': 1,
                    'print_iteration': False,
                    'print_maxit_reached': False,
                    'print_header': False,
                    'verbose_init': False,
                    'print_time': 0,
                    'opttol': 1e-4,
                    }
        self.opts.update(rti_opts)
