from .nlpsol import NlpsolSolver
from horizon.problem import Problem
from typing import Dict
import casadi as cs

class IpoptSolver(NlpsolSolver):

    def __init__(self, prb: Problem, opts: Dict) -> None:
        filtered_opts = None 
        if opts is not None:
            filtered_opts = {k: opts[k] for k in opts.keys() if k.startswith('ipopt.')}
        super().__init__(prb, opts=filtered_opts, solver_plugin='ipopt')
