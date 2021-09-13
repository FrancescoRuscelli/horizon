from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from horizon.utils.casadi_kin_dyn import InverseDynamics

class ModelHandler:
    def __init__(self):
        self.kindyn = None

    def setModel(self, urdf):
        self.kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

    def getKinDyn(self):
        return self.kindyn

    def computeID(self, q, q_dot, q_ddot):
        if not self.kindyn:
            raise Exception('kindyn is not setted. Cannot compute inverse dynamics.')

        tau = InverseDynamics(self.kindyn).call(q, q_dot, q_ddot)

    def getNq(self):
        if not self.kindyn:
            raise Exception('kindyn is not setted. Cannot get dimension of q.')

        return self.kindyn.nq()

    def getNqdot(self):
        if not self.kindyn:
            raise Exception('kindyn is not setted. Cannot get dimension of qdot.')

        return self.kindyn.nv()
