from horizon.problem import Problem
from typing import Dict, Iterable
from abc import ABC, abstractmethod

class Solver(ABC):

    """
    Solver is an abstract interface for a generic solver
    that aims to find a solution of an instance of
    horizon.problem.Problem.
    """

    def __init__(self, 
                 prb: Problem, 
                 dt: float, 
                 opts: Dict = None) -> None:
        """
        Construct a solver

        Args:
            prb (Problem): the horizon's problem instance to be solved
            dt (float): the discretization step
            opts (Dict, optional): A solver-dependent Dict of options. Defaults to None.
        """
        
        # handle inputs
        if opts is None:
            opts = dict()

        self.opts = opts
        self.prb = prb
        self.dt = dt

        # save state and control
        self.x = prb.getState().getVars()
        self.u = prb.getInput().getVars()
        self.xdot = prb.getDynamics()
        

    @abstractmethod
    def solve(self) -> bool:
        """
        Solve the horizon's problem

        Returns:
            bool: success flag
        """
        pass 
