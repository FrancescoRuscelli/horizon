from .solver import Solver
from horizon.problem import Problem
from typing import Dict
import casadi as cs

class BlockSqpSolver(Solver):

    def __init__(self, prb: Problem, dt: float, opts: Dict) -> None:
        
        super().__init__(prb, dt, opts=opts)
        
        # generate problem to be solver
        prb.createProblem()

        # create solver from prob
        self.solver = cs.nlpsol('blocksqp_solver', 'blocksqp', prb.prob, opts)

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


    def solve(self) -> bool:
        # update bounds and initial guess
        w0, lbw, ubw, lbg, ubg, p = self.prb.updateProblem()

        # solve
        sol = self.solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=p)

        # retrieve state and input trajector
        input_vars = [v.tag for v in self.prb.getInput().var_list]
        state_vars = [v.tag for v in self.prb.getState().var_list]
        pos = 0

        # loop over nodes
        for node, val in self.prb.var_container.getVarImplDict().items():

            
            # variables which don't depend on the node (ignore for now)
            if node == 'nNone':
                # note: for now, we only focus on state and input!!
                for name, var in val.items():
                    dim = var['var'].shape[0]
                    pos += dim  
                continue
            
            node_number = int(node[node.index('n') + 1:])

            # for loop handling state vars
            for name, var in val.items():
                if name not in state_vars:
                    continue
                off, _ = self.prb.getState().getVarIndex(name)
                var_nodes = self.prb.var_container.getVarAbstrDict(offset=False)[name].getNodes()
                if node_number in var_nodes:
                    dim = var['var'].shape[0]
                    self.x_opt[off:off+dim, node_number - var_nodes[0]] = sol['x'][pos:pos + dim].full().flatten()
                    pos += dim

            # for loop handling input vars (pretty ugly: todo refactor)
            for name, var in val.items():
                if name not in input_vars:
                    continue
                off, _ = self.prb.getInput().getVarIndex(name)
                var_nodes = self.prb.var_container.getVarAbstrDict(offset=False)[name].getNodes()
                if node_number in var_nodes:
                    dim = var['var'].shape[0]
                    self.u_opt[off:off+dim, node_number - var_nodes[0]] = sol['x'][pos:pos + dim].full().flatten()
                    pos += dim
            
        return True
