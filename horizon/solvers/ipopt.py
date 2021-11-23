from .nlpsol import NlpsolSolver
from horizon.problem import Problem
from typing import Dict
import casadi as cs

class IpoptSolver(NlpsolSolver):

    def __init__(self, prb: Problem, opts: Dict) -> None:
        
        super().__init__(prb, opts=opts, solver_plugin='ipopt')
