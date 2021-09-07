import sys
from functools import partial

from PyQt5.QtWidgets import QGridLayout, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QCompleter

from horizon_gui.custom_functions.highlighter import Highlighter
from PyQt5.QtGui import QPalette, QFont, QColor
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot

from horizon_gui.gui.widget1_ui import Ui_HorizonGUI
from horizon_gui.gui.variables_module.variables_gui import VariablesGui
from horizon_gui.gui.functions_module.functions_gui import FunctionsGui
from horizon_gui.gui.transcriptions_module.transcriptions_gui import TranscriptionGui
from horizon_gui.gui.dynamics_module.dynamics_gui import DynamicsGui
from horizon_gui.gui.model_module.model_gui import ModelGui
from horizon_gui.gui.problem_module.problem_gui import ProblemGui
from horizon_gui.custom_widgets import horizon_line
from horizon_gui.custom_functions.txt_to_fun import TxtToFun
from horizon_gui.definitions import CSS_DIR
from horizon_gui.gui.gui_receiver import horizonImpl



class MainInterface(QWidget, Ui_HorizonGUI):
    generic_sig = pyqtSignal(str)

    def __init__(self, horizon_receiver: horizonImpl, logger=None):
        super(MainInterface, self).__init__()

        self.setupUi(self)
        # #todo logger! use it everywhere!

        self.logger = logger
        self.logger.addHandler(self.consoleLogger)

        # emitting stream from python terminal
        # sys.stdout = EmittingStream()
        # sys.stdout.textWritten.connect(self.normalOutputWritten)

        # todo put it somewhere else?
        # self._init()
        # self.horizon_receiver = None
        self.horizon_receiver = horizon_receiver
        self.nodes = horizon_receiver.getNodes()


        # highlighter
        self.highlighter = self.setupHighlighter()
        # completer
        self.completer = self.setupCompleter()

        # variables
        self.variables_gui = VariablesGui(self.horizon_receiver, self.logger, self)
        var_box_layout = QVBoxLayout()
        self.varBox.setLayout(var_box_layout)
        var_box_layout.addWidget(self.variables_gui)

        # functions
        # todo remove in some ways highlighter and completer from here?
        self.functions_gui = FunctionsGui(self.horizon_receiver, self.highlighter, self.completer, self.logger, self)
        fun_box_layout = QVBoxLayout()
        self.funBox.setLayout(fun_box_layout)
        fun_box_layout.addWidget(self.functions_gui)

        # transcriptions methods
        self.trans_gui = TranscriptionGui(self.horizon_receiver, self.logger, self)
        transcription_box_layout = QVBoxLayout()
        self.TranscriptionBox.setLayout(transcription_box_layout)
        transcription_box_layout.addWidget(self.trans_gui)

        # dynamics
        self.dyn_gui = DynamicsGui(self.horizon_receiver, self.logger, self)
        dynamics_box_layout = QVBoxLayout()
        self.DynamicsBox.setLayout(dynamics_box_layout)
        dynamics_box_layout.addWidget(self.dyn_gui)
        # model
        self.model_gui = ModelGui(self.logger, self)
        model_box_layout = QVBoxLayout()
        self.ModelBox.setLayout(model_box_layout)
        model_box_layout.addWidget(self.model_gui)

        # problem
        self.problem_gui = ProblemGui(self.horizon_receiver, self.logger, self)
        problem_box_layout = QVBoxLayout()
        self.PrbBox.setLayout(problem_box_layout)
        problem_box_layout.addWidget(self.problem_gui)
        # spinbox to set offset variables
        # self.varOffsetInput.setRange(-N, 0)

        #
        # with open(CSS_DIR + '/button_old.css', 'r') as f:
        #     self.SolveButton.setStyleSheet(f.read())
        #
        # with open(CSS_DIR + '/button_old.css', 'r') as f:
        #     self.PlotButton.setStyleSheet(f.read())

        self.NodesSpinBox.valueChanged.connect(self.setBoxNodes)
        self.CreateButton.clicked.connect(self.createButtonPushed)
        self.SolveButton.clicked.connect(self.solveButtonPushed)
        self.PlotButton.clicked.connect(self.plotButtonPushed)

        # these set to NOT READY the create/solve problem buttons if something in the horizon problem is changed
        self.problem_gui.constraint_line.active_fun_horizon.connect(partial(self.ledCreate.setReady, False))
        self.problem_gui.constraint_line.active_fun_horizon.connect(partial(self.ledSolve.setReady, False))
        self.problem_gui.constraint_line.bounds_changed.connect(partial(self.ledCreate.setReady, False))
        self.problem_gui.constraint_line.bounds_changed.connect(partial(self.ledSolve.setReady, False))
        self.problem_gui.cost_line.active_fun_horizon.connect(partial(self.ledCreate.setReady, False))
        self.problem_gui.cost_line.active_fun_horizon.connect(partial(self.ledSolve.setReady, False))
        self.problem_gui.cost_line.bounds_changed.connect(partial(self.ledCreate.setReady, False))
        self.problem_gui.cost_line.bounds_changed.connect(partial(self.ledSolve.setReady, False))
        self.NodesSpinBox.valueChanged.connect(partial(self.ledCreate.setReady, False))
        self.NodesSpinBox.valueChanged.connect(partial(self.ledSolve.setReady, False))
        self.problem_gui.constraint_line.function_nodes_changed.connect(partial(self.ledCreate.setReady, False))
        self.problem_gui.constraint_line.function_nodes_changed.connect(partial(self.ledSolve.setReady, False))

        self.ledSolve.setEnabled(False)

        # TRANSCRIPTION STUFF
        # the transcription need to be updated if nodes changed
        self.NodesSpinBox.valueChanged.connect(self.trans_gui.updateTranscriptionMethod)

        # MODEL STUFF
        # self.model_gui.modelLoaded.connect(self.writeInStatusBar('Opening Horizon problem!'))

        # VARIABLES STUFF
        # the dynamics becomes not ready if new state or input variables are inserted
        self.variables_gui.stateVarAdded.connect(self.dyn_gui.manageDisplay)
        self.variables_gui.stateVarAdded.connect(self.updateHighlighter)
        self.variables_gui.stateVarAdded.connect(partial(self.ledCreate.setReady, False))
        self.variables_gui.stateVarAdded.connect(partial(self.ledSolve.setReady, False))
        self.variables_gui.genericSignal.connect(self.on_generic_sig)

        # FUNCTIONS STUFF
        # self.functions_gui.funAdded.connect(#something)

        # when opening horizon, fill the GUI
        for name, data in horizon_receiver.getVarDict().items():
            self.variables_gui.addVariableToGui(name)

        for name, data in horizon_receiver.getFunctionDict().items():
            self.addFunctionToGUI(name, data['str'])
            if data['active'] is not None:
                if data['active'].getType() == 'constraint':
                    line = self.constraintLine
                    # todo getDim() only taking the row, what if it's a matrix?
                    line.addFunctionToSingleLine(name, data['active'].getDim()[0])
                    line.addFunctionToMultiLine(name)
                elif data['active'].getType() == 'costfunction':
                    line = self.costfunctionLine
                    line.addFunctionToSingleLine(name, data['active'].getDim()[0])
                    line.addFunctionToMultiLine(name)


        self.initDummyStuff()

    def initDummyStuff(self):

        n_nodes = 30
        self.NodesSpinBox.setValue(n_nodes)
        self.setBoxNodes(n_nodes)
        # urdf_file = '/home/francesco/hhcm_workspace/src/horizon/horizon/examples/urdf/cart_pole.urdf'
        urdf_file = '/home/francesco/catkin_ws/external/casadi_horizon/horizon/examples/urdf/cart_pole.urdf'
        self.model_gui.loadModel(urdf_file)

        state_vars = dict(q=2, q_dot=2)
        input_vars = dict(q_ddot=2)

        # variables
        for name, dim in state_vars.items():
            flag, signal = self.horizon_receiver.createVariable('State', name, dim, 0)
            if flag:
                self.variables_gui.addVariableToGui(name)
                self.updateHighlighter(name)

        for name, dim in input_vars.items():
            flag, signal = self.horizon_receiver.createVariable('Input', name, dim, 0)
            if flag:
                self.variables_gui.addVariableToGui(name)
                self.updateHighlighter(name)

        # Limits
        import numpy as np
        self.variables_gui.on_var_lb_changed('q', range(self.nodes+1), [-0.5, -2. * np.pi])
        self.variables_gui.on_var_ub_changed('q', range(self.nodes+1), [0.5, 2. * np.pi])
        self.variables_gui.on_var_lb_changed('q', 0, [0., 0.])
        self.variables_gui.on_var_ub_changed('q', 0, [0., 0.])

        self.variables_gui.on_var_lb_changed('q_dot', range(self.nodes+1), [-100., -100.])
        self.variables_gui.on_var_ub_changed('q_dot', range(self.nodes+1), [100., 100.])
        self.variables_gui.on_var_lb_changed('q_dot', 0, [0., 0.])
        self.variables_gui.on_var_ub_changed('q_dot', 0, [0., 0.])

        self.variables_gui.on_var_lb_changed('q_ddot', range(self.nodes+1), [-1000., -1000.])
        self.variables_gui.on_var_ub_changed('q_ddot', range(self.nodes+1), [1000., 1000.])
        self.variables_gui.on_var_lb_changed('q_ddot', 0, [0., 0.])
        self.variables_gui.on_var_ub_changed('q_ddot', 0, [0., 0.])

        # # Intial guesses
        self.variables_gui.on_var_ig_changed('q', range(self.nodes+1), [0., 0.])
        self.variables_gui.on_var_ig_changed('q_dot', range(self.nodes+1), [0., 0.])
        self.variables_gui.on_var_ig_changed('q_ddot', range(self.nodes+1), [0., 0.])

        # # Dynamics
        dyn = 'cs.vertcat(q_dot, q_ddot)'
        self.horizon_receiver.setDynamics('custom', dyn)
        self.dyn_gui.display.setText('custom')
        self.dyn_gui.display.setReady(True)

        # # Transcription
        type = 'multiple_shooting' # direct_collocation
        opts = dict(integrator='RK4') #3
        self.horizon_receiver.setTranscriptionMethod(type, opts)

        self.trans_gui.display.setText(type)
        self.trans_gui.display.setReady(True)

        # # Cost function
        cost_name = 'minimize_q_ddot'
        str_fun = 'cs.sumsqr(q_ddot)'
        flag = self.horizon_receiver.addFunction(dict(name=cost_name, str=str_fun, active=None))
        if flag:
            self.functions_gui.addFunctionToGui(cost_name, str_fun)

        # prb.createIntermediateCost("qddot", cs.sumsqr(qddot))
        # # Constraint
        # prb.createFinalConstraint("up", q[1] - np.pi)
        # prb.createFinalConstraint("final_qdot", qdot)


    def setupHighlighter(self):
        self.fun_keywords = list()
        # with QLineEdit doesn't work, so I had to override QTextEdit

        highlighter = Highlighter()
        # adding math operators to highlighter
        math_operators = TxtToFun.getValidOperators()
        highlighter.addOperators(math_operators)
        return highlighter
    def setupCompleter(self):
        completer = QCompleter(self.fun_keywords)
        completer.setCaseSensitivity(Qt.CaseInsensitive)
        completer.setWrapAround(False)
        return completer

    def updateHighlighter(self, var_name):
        # add variable to highlighter and to completers
        self.highlighter.addKeyword(var_name)
        self.fun_keywords.append('{}'.format(var_name))
        model = self.completer.model()
        model.setStringList(self.fun_keywords)

    def createButtonPushed(self):
        if self.horizon_receiver.generate():
            self.SolveButton.setEnabled(True)
            self.ledSolve.setEnabled(True)
            self.ledCreate.setReady(True)
        else:
            self.logger.warning('Failed to generate problem.')
            # with open(CSS_DIR + '/button_new.css', 'r') as f:
            #     self.SolveButton.setStyleSheet(f.read())

    def solveButtonPushed(self):
        if self.horizon_receiver.solve():
            self.PlotButton.setEnabled(True)
            self.ledSolve.setReady(True)
            # with open(CSS_DIR + '/button_new.css', 'r') as f:
            #     self.PlotButton.setStyleSheet(f.read())

    def plotButtonPushed(self):
        self.horizon_receiver.plot()

    @pyqtSlot()
    def on_generic_sig(self, str):
        self.generic_sig.emit(str)

    def setBoxNodes(self, n_nodes):
        self.nodes = n_nodes
        self.horizon_receiver.setHorizonNodes(n_nodes)  # setting to casadi the new number of nodes
        self.problem_gui.constraint_line.setHorizonNodes(n_nodes + 1)  # n_nodes+1 to account for the final node
        self.problem_gui.cost_line.setHorizonNodes(n_nodes + 1)  # n_nodes+1 to account for the final node

    def _connectActions(self):
        pass
        # todo PUTT ALL OTHER CONNECT
        # self.SVAddButton.clicked.connect(self.generateStateVariable)

    def __del__(self):
        # Restore sys.stdout
        self.logger.removeHandler(self.consoleLogger)
        sys.stdout = sys.__stdout__
