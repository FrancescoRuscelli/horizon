import logging

import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib import gridspec
from horizon.problem import Problem
import math
import numpy as np
import casadi as cs

class PlotterHorizon:
    def __init__(self, prb: Problem, logger=None):

        self.prb = prb
        self.logger = logger

    def _createPlotGrid(self, n_rows_max, n_plots, title):

        cols = n_plots if n_plots < n_rows_max else n_rows_max
        rows = int(math.ceil(n_plots / cols))

        gs = gridspec.GridSpec(rows, cols)
        fig = plt.figure()
        fig.suptitle(title)

        return fig, gs

    def _plotVar(self, val, ax, abstract_var):
        for dim in range(val.shape[0]):
            baseline, = ax.plot(range(val.shape[1]), val[dim, :], marker="o")
            lb, ub = abstract_var.getBounds()
            ax.plot(range(val.shape[1]), lb[dim, :], marker="x", color=baseline.get_color())
            ax.plot(range(val.shape[1]), ub[dim, :], marker="x", color=baseline.get_color())


    def plotVariables(self):

        sol = self.prb.getSolution()
        if sol is None:
            raise Exception('Problem must contain a solution.')

        fig, gs = self._createPlotGrid(3, len(sol), 'Variables')
        i = 0
        for key, val in sol.items():
            ax = fig.add_subplot(gs[i])
            self._plotVar(val, ax, self.prb.getVariables(key))

            # options
            ax.set_title('{}'.format(key))
            ax.ticklabel_format(useOffset=False, style='plain')
            ax.yaxis.set_major_formatter(FormatStrFormatter('%g'))
            # ax.set(xlabel='nodes', ylabel='vals')
            plt.xticks(list(range(val.shape[1])))
            i = i+1

        fig.tight_layout()
        plt.show(block=False)

    def plotVariable(self, name):

        sol = self.prb.getSolution()
        if sol is None:
            raise Exception('Problem must contain a solution.')

        val = sol[name]

        fig, ax = plt.subplots()
        self._plotVar(val, ax, prb.getVariables(name))

        ax.set_title('{}'.format(name))
        plt.xticks(list(range(val.shape[1])))
        ax.set(xlabel='nodes', ylabel='vals')

    def plotFunctions(self):

        sol = self.prb.getSolution()
        if sol is None:
            raise Exception('Problem must contain a solution.')

        fig, gs = self._createPlotGrid(3, len(self.prb.getConstraints()), 'Functions')

        i = 0
        for name, fun in self.prb.getConstraints().items():
            ax = fig.add_subplot(gs[i])
            fun_evaluated = self.prb.evalFun(fun)
            self._plotVar(fun_evaluated, ax, self.prb.getConstraints(name))

            ax.set_title('{}'.format(name))
            plt.xticks(list(range(fun_evaluated.shape[1])))
            ax.ticklabel_format(useOffset=False, style='plain')
            ax.yaxis.set_major_formatter(FormatStrFormatter('%g'))
            i = i+1

        fig.tight_layout()
        plt.show(block=False)

if __name__ == '__main__':

    nodes = 10
    prb = Problem(nodes, logging_level=logging.DEBUG)
    x = prb.createStateVariable('x', 1)
    v = prb.createStateVariable('v', 1)
    k = prb.createVariable('k', 1, range(2, 8))
    u = prb.createInputVariable('u', 1)
    t_tot = prb.createVariable('t', 1)
    t_tot.setBounds([100], [100])


    # danieli = prb.createConstraint('danieli', x-k, nodes=range(2, 8))
    diosporco = prb.createConstraint('diosporcomaledetto', x - t_tot)
    diosporco.setBounds([100], [100])
    xprev = x.getVarOffset(-1)

    xprev_copy = x.getVarOffset(-1)
    xnext = x.getVarOffset(+1)

    opts = {'ipopt.tol': 1e-4,
            'ipopt.max_iter': 2000}

    prb.createProblem(opts=opts)

    print(prb.getProblem()['x'])

    prb.solveProblem()



    hplt = PlotterHorizon(prb)
    hplt.plotVariables()
    hplt.plotFunctions()

    exit()

    nodes = 5
    prb = Problem(nodes, logging_level=logging.DEBUG)
    x = prb.createStateVariable('x', 2)
    # y = prb.createStateVariable('y', 2)
    t = prb.createVariable('t', 2)
    p = prb.createVariable('p', 2)
    x.setInitialGuess([1, 1])
    x.setBounds([1, 1], [2, 2])
    x.setBounds([1, 1], [1, 1], nodes=3)
    danieli = prb.createConstraint('danieli', x*t)


    danieli.setBounds([10, 10], [20, 20])

    prb.createProblem()
    sol = prb.solveProblem()


    hplt = PlotterHorizon(prb)
    hplt.plotVariables()
    hplt.plotFunctions()

    exit()




    nodes = 8
    prb = Problem(nodes, logging_level=logging.DEBUG)
    x = prb.createStateVariable('x', 2)
    y = prb.createStateVariable('y', 2)

    danieli = prb.createConstraint('danieli', x + y)
    suka = prb.createConstraint('suka', x[0] * y[1])

    x.setBounds([2, 2], [2, 2])


    danieli.setBounds([12, 12], [12, 12], 4)
    suka.setBounds([4], [4], [2, 3, 5, 6])


    prb.createProblem()
    sol = prb.solveProblem()

    print('==============================================================')

    hplt = PlotterHorizon(sol)

    hplt.plotVariables()
    # hplt.plotVariable('x')
