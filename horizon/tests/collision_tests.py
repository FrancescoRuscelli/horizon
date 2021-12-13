import unittest

from horizon.utils import collision
import casadi as cs
import rospkg
import os
import numpy as np
np.set_printoptions(suppress=True, precision=3)

from casadi_kin_dyn import pycasadi_kin_dyn as cas_kin_dyn
from urdf_parser_py import urdf

class CollisionTest(unittest.TestCase):

    def setUp(self) -> None:

        # load urdf
        urdffile = os.path.join(rospkg.RosPack().get_path('teleop_urdf'), 'urdf', 'teleop_capsules.rviz')
        with open(urdffile, 'r') as f:
            urdfstr = f.read()

        self.coll = collision.CollisionHandler(urdfstr, cas_kin_dyn.CasadiKinDyn(urdfstr))

        g1 = urdf.Cylinder(length=0.20, radius=0.05)
        c1 = urdf.Collision(geometry=g1, origin=urdf.Pose(xyz=[0.1, 0, 0], rpy=[0, np.pi/2, 0]))
        self.coll.world['world/ciao'] = c1

    def test_robot(self):
        nq = self.coll.kindyn.nq()
        q = cs.SX.sym('q', nq)
        d = self.coll.compute_distances(q)
        self.assertEqual(d.size1(), len(self.coll.capsules)*len(self.coll.world))

        dfun = cs.Function('dfun', [q], [d], ['q'], ['d'])
        dvalue = dfun.jacobian()(q=np.array([0, 1, 1, 0, 0]))
        print(dvalue)

    def test_parallel_capsules(self):
        g1 = urdf.Cylinder(length=0.20, radius=0.05)
        c1 = urdf.Collision(geometry=g1, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, np.pi/2, 0]))

        g2 = urdf.Cylinder(length=0.20, radius=0.05)
        c2 = urdf.Collision(geometry=g2, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, np.pi/2, 0]))

        T1 = np.eye(4)
        T1[0:3, 3] = [-0.3, 0, 0]

        T2 = np.eye(4)
        T2[0:3, 3] = [0.3, 0, 0]

        ret = collision.CollisionHandler.dist_capsule_capsule(c1, cs.DM(T1), c2, cs.DM(T2))
        self.assertAlmostEqual(ret, 0.3)

    def test_touching_capsules(self):
        g1 = urdf.Cylinder(length=0.20, radius=0.05)
        c1 = urdf.Collision(geometry=g1, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, np.pi/2, 0]))

        g2 = urdf.Cylinder(length=0.20, radius=0.05)
        c2 = urdf.Collision(geometry=g2, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, np.pi/2, 0]))

        T1 = np.eye(4)
        T1[0:3, 3] = [-0.15, 0, 0]

        T2 = np.eye(4)
        T2[0:3, 3] = [0.15, 0, 0]

        ret = collision.CollisionHandler.dist_capsule_capsule(c1, cs.DM(T1), c2, cs.DM(T2))
        self.assertAlmostEqual(ret, 0.0)

    def test_touching_capsules_rot(self):
        g1 = urdf.Cylinder(length=0.20, radius=0.05)
        c1 = urdf.Collision(geometry=g1, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0]))

        g2 = urdf.Cylinder(length=0.20, radius=0.05)
        c2 = urdf.Collision(geometry=g2, origin=urdf.Pose(xyz=[0, 0, 0], rpy=[0, np.pi/2, 0]))

        T1 = np.eye(4)
        T1[0:3, 3] = [-0.05, 0, 0]

        T2 = np.eye(4)
        T2[0:3, 3] = [0.15, 0, 0]

        ret = collision.CollisionHandler.dist_capsule_capsule(c1, cs.DM(T1), c2, cs.DM(T2))
        self.assertAlmostEqual(ret, 0.0)


if __name__ == '__main__':
    unittest.main()