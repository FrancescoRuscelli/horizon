import logging

import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib import gridspec
from horizon.problem import Problem
import math

class PlotterHorizon:
    def __init__(self, sol=None, logger=None):

        self.sol = sol
        self.logger = logger

    def setSolution(self, sol):
        self.sol = sol

    def plotVariables(self):

        if self.sol is None:
            if self.logger:
                self.logger("Plotter is empty. Load a solution.")
            return 0

        n_col = 3
        n_plots = len(self.sol)
        cols = n_col if n_col < len(self.sol) else len(self.sol)
        rows = int(math.ceil(n_plots / cols))

        gs = gridspec.GridSpec(rows, cols)
        fig = plt.figure()
        i = 0
        for key, val in self.sol.items():
            ax = fig.add_subplot(gs[i])
            for dim in range(val.shape[0]):
                ax.plot(range(val.shape[1]), val[dim, :], marker="o")

            ax.set_title('{}'.format(key))
            ax.ticklabel_format(useOffset=False, style='plain')
            # ax.yaxis.set_major_formatter(FormatStrFormatter('%g'))
            # ax.set(xlabel='nodes', ylabel='vals')
            plt.xticks(list(range(val.shape[1])))
            i = i+1



        fig.tight_layout()
        plt.show()

    def plotVariable(self, name):

        val = self.sol[name]

        fig, ax = plt.subplots()
        for dim in range(val.shape[0]):
            ax.plot(range(self.nodes + 1), val[dim, :], marker="o")
        ax.set_title('{}'.format(name))
        plt.xticks(list(range(self.nodes + 1)))
        ax.set(xlabel='nodes', ylabel='vals')

        plt.show()


if __name__ == '__main__':
    nodes = 1
    prb = Problem(nodes, logging_level=logging.DEBUG)
    x = prb.createStateVariable('x', 2)
    # y = prb.createStateVariable('y', 2)
    t = prb.createVariable('t', 2)
    p = prb.createVariable('p', 2)
    x.setInitialGuess([1, 1])

    danieli = prb.createConstraint('danieli', x*t)

    danieli.setBounds([10, 10], [10, 10])

    prb.createProblem({"nlpsol.ipopt":True})
    sol = prb.solveProblem()

    print(sol)
    hplt = PlotterHorizon(sol)
    hplt.plotVariables()

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


    prb.createProblem({"nlpsol.ipopt":True})
    sol = prb.solveProblem()

    print('==============================================================')

    hplt = PlotterHorizon(sol)

    hplt.plotVariables()
    # hplt.plotVariable('x')
