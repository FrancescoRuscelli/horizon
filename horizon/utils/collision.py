from urdf_parser_py import urdf
import casadi as cs
from scipy.spatial.transform import Rotation as R

class CollisionHandler:

    def __init__(self, urdfstr, kindyn) -> None:
        self.urdf = urdf.Robot.from_xml_string(urdfstr)
        self.capsules = dict()
        self.world = dict()
        self.kindyn = kindyn
        self.fk = dict()

        for l in self.urdf.links:
            caps = CollisionHandler.collision_to_capsule(l)
            if caps is not None:
                print(f'found capsule for link {l.name}')
                self.capsules[l.name] = caps
                self.fk[l.name] = cs.Function.deserialize(self.kindyn.fk(l.name))

    def get_function(self):
        q = cs.SX.sym('q', self.kindyn.nq())
        d = self.compute_distances(q)
        return cs.Function('collision', [q], [d], ['q'], ['d'])
    
    def compute_distances(self, q):

        dlist = list()
        
        # loop over body collision
        for bname, bc in self.capsules.items():
            ee_pos, ee_rot = self.fk[bname](q)
            Tb = cs.SX.eye(4)
            Tb[0:3, 0:3] = ee_rot 
            Tb[0:3, 3] = ee_pos

            # loop over world collisions
            for wcname, wc in self.world.items():
                d = CollisionHandler.dist_capsule_capsule(bc, Tb, wc, cs.DM.eye(4))
                dlist.append(d)
            
        return cs.vertcat(*dlist)
    

    @classmethod
    def collision_to_capsule(cls, link: urdf.Link):
        """[summary]

        Args:
            link (urdf.Link): [description]

        Returns:
            [type]: [description]
        """
        
        if len(link.collisions) != 3:
            return None
        
        coll_geom = [c.geometry for c in link.collisions]
        cylinders = [c for c in link.collisions if isinstance(c.geometry, urdf.Cylinder)]
        spheres = [c for c in link.collisions if isinstance(c.geometry, urdf.Sphere)]

        if len(cylinders) == 1 and len(spheres) == 2:
            return cylinders[0]

    
    @classmethod
    def dist_capsule_capsule(cls, capsule_1: urdf.Collision, T1, capsule_2: urdf.Collision, T2):
        p_FC1o = cs.mtimes(T1[0:3, 0:3], capsule_1.origin.xyz) + T1[0:3,3]
        p_FC2o = cs.mtimes(T2[0:3, 0:3], capsule_2.origin.xyz) + T2[0:3,3]

        # A capsule is defined centered on the origin of its canonical frame C
        # and with the central line segment aligned with Cz. So, the two end points
        # of the capsule's center line segment are at `z+ = lz / 2 * Cz` and
        # `z- = -lz / 2 * Cz`, respectively. Cz_F is simply the third column of the
        # rotation matrix, R_FC. This "half arm" is the position vector from the
        # canonical frame's origin to the z+ point: p_CoZ+_F in frame F.
        def calc_half_arm(capsule, T):
            half_length = capsule.geometry.length / 2.
            rot = R.from_euler(seq='xyz', angles=capsule.origin.rpy).as_matrix()
            Cz_F = (cs.mtimes(T[0:3, 0:3], rot))[:,2]
            return half_length * Cz_F
        
        half_arm_1_F = calc_half_arm(capsule_1, T1)
        p_FC1a = p_FC1o + half_arm_1_F
        p_FC1b = p_FC1o - half_arm_1_F
        half_arm_2_F = calc_half_arm(capsule_2, T2)
        p_FC2a = p_FC2o + half_arm_2_F
        p_FC2b = p_FC2o - half_arm_2_F

        result = cls.dist_segment_segment(p_FC1a, p_FC1b, p_FC2a, p_FC2b)

        squared_dist = result[6]
        p_FN1 = result[0:3]
        p_FN2 = result[3:6]

        segment_dist = cs.sqrt(squared_dist)
        dist = segment_dist - capsule_1.geometry.radius - capsule_2.geometry.radius

        return dist

    @classmethod
    def dist_segment_segment(cls, p_FP1, p_FQ1, p_FP2, p_FQ2):

        kEps = 1e-8
        kEpsSquared = kEps**2

        p_P1Q1 = p_FQ1 - p_FP1  # Segment 1's displacement vector: D1.
        p_P2Q2 = p_FQ2 - p_FP2  # Segment 2's displacement vector: D2.
        p_P2P1 = p_FP1 - p_FP2

        a = cs.dot(p_P1Q1, p_P1Q1)  # Squared length of segment S1, always nonnegative.
        e = cs.dot(p_P2Q2, p_P2Q2)  # Squared length of segment S2, always nonnegative.
        f = cs.dot(p_P2Q2, p_P2P1)
        c = cs.dot(p_P1Q1, p_P2P1)

        def if_points():
            return cs.vertcat(p_FP1, p_FP2, cs.sumsqr(p_FP1-p_FP2))

        def if_not_points():
            
            def if_first_segment_is_point():
                s = 0.0
                t = cls.clamp(f/e, 0.0, 1.0)
                p_FC1 = p_FP1
                p_FC2 = p_FP2 + t*p_P2Q2
                return cs.vertcat(p_FC1, p_FC2, cs.sumsqr(p_FC1-p_FC2))

            def if_second_segment_is_point():
                s = cls.clamp(-c/a, 0.0, 1.0)
                t = 0.0
                p_FC1 = p_FP1 + s*p_P1Q1
                p_FC2 = p_FP2 
                return cs.vertcat(p_FC1, p_FC2, cs.sumsqr(p_FC1-p_FC2))

            def if_not_degenerate():
                b = cs.dot(p_P1Q1, p_P2Q2)
                denom = cs.if_else(a*e-b*b > 0, a*e-b*b, 0)
                s = cs.if_else(denom > kEpsSquared, 
                               cls.clamp((b*f - c*e)/denom, 0.0, 1.0),
                               0.0)
                t = (b*s + f) / e

                def if_t_in_0_1():
                    p_FC1 = p_FP1 + s*p_P1Q1
                    p_FC2 = p_FP2 + t*p_P2Q2
                    return cs.vertcat(p_FC1, p_FC2, cs.sumsqr(p_FC1-p_FC2))

                def if_t_negative():
                    t = 0
                    s = cls.clamp(-c/a, 0.0, 1.0)
                    p_FC1 = p_FP1 + s*p_P1Q1
                    p_FC2 = p_FP2 
                    return cs.vertcat(p_FC1, p_FC2, cs.sumsqr(p_FC1-p_FC2))

                def if_t_gt_1():
                    t = 1.0
                    s = cls.clamp((b - c)/a, 0.0, 1.0)
                    p_FC1 = p_FP1 + s*p_P1Q1
                    p_FC2 = p_FP2 + t*p_P2Q2
                    return cs.vertcat(p_FC1, p_FC2, cs.sumsqr(p_FC1-p_FC2))

                return cs.if_else(cs.logic_and(t > 0, t < 1), 
                                  if_t_in_0_1(), 
                                  cs.if_else(t < 0, 
                                             if_t_negative(), 
                                             if_t_gt_1()))
            
            return cs.if_else(a < kEpsSquared,
                            if_first_segment_is_point(),
                            cs.if_else(e < kEpsSquared, 
                                       if_second_segment_is_point(),
                                       if_not_degenerate()))


        return cs.if_else(cs.logic_and(a <= kEpsSquared, e <= kEpsSquared), 
                          if_points(), 
                          if_not_points())

                          

    
    @classmethod
    def clamp(cls, x, xmin, xmax):
        return cs.if_else(x < xmin, 
                          xmin, 
                          cs.if_else(x > xmax, xmax, x)
                          )
