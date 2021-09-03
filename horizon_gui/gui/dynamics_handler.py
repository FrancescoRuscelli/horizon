from horizon.problem import Problem
from horizon.utils import utils
from logging import Logger
class DynamicsHandler:
    def __init__(self, prob: Problem, logger: Logger):

        self.prob = prob
        self.logger = logger
        self.default_dyn_list = ['vertcat_state', 'double_integrator', 'double_integrator_fb']


    def set_custom_dynamics(self, dyn):
        try:
            self.prob.setDynamics(dyn)
            self.logger.info(f'Custom dynamics successfully set: {self.prob.getDynamics()}')
        except Exception as e:
            if self.logger:
                self.logger.warning(f'DynamicsHandler, set_custom_dynamics: {e}')

    def set_default_dynamics(self, dyn):
        try:
            maker = self.get_dyn_default(dyn)
            maker()
            self.logger.info(f'Default dynamics successfully set: {self.prob.getDynamics()}')
        except Exception as e:
            if self.logger:
                self.logger.warning(f'DynamicsHandler, set_default_dynamics: {e}')

    def getList(self):
        return self.default_dyn_list

    def get_dyn_default(self, method):
        if method == 'vertcat_state':
            return self.set_vertcat_state
        elif method == 'double_integrator':
            return self.set_double_integrator
        elif method == 'double_integrator_fb':
            return self.set_double_integrator_fb

    def set_vertcat_state(self):
        self.prob.setDynamics(self.prob.getState().getVars())

    def set_double_integrator(self):
        # yet to do
        # utils.double_integrator(q, qdot, qddot)
        pass

    def set_double_integrator_fb(self):
        # yet to do
        # x, xdot = utils.double_integrator_with_floating_base(q, qdot, qddot)
        pass

if __name__ == '__main__':
    ab = Problem(2)

    dyn = DynamicsHandler(ab)
    dyn.set_dynamics('vertcat_state')

    # DynamicsHandler.set_dynamics(ab, 'vertcat_state')