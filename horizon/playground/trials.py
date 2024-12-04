import numpy as np
from horizon.problem import Problem


# f, c, cdot, # forces, contacts, com_vel
# initial_foot_position[0][2].__float__(), z_foot
# c_ref, #
# ns, 20
# number_of_legs=number_of_legs, 2
# contact_model=contact_model, 4 (points on the contact
# max_force=max_contact_force, # 1000
# max_velocity=max_contact_velocity 10#

class steps_phase:
    def __init__(self, f, c, cdot, c_init_z, c_ref, nodes=20, number_of_legs=2, contact_model=4, max_force=1000.,
                 max_velocity=10.):
        self.f = f
        self.c = c
        self.cdot = cdot
        self.c_ref = c_ref

        self.number_of_legs = number_of_legs
        self.contact_model = contact_model

        self.nodes = nodes
        self.step_counter = 0

        # JUMP
        self.jump_c = []
        self.jump_cdot_bounds = []
        self.jump_f_bounds = []
        sin = 0.1 * np.sin(np.linspace(0, np.pi, 8))
        for k in range(0, 7):  # 7 nodes down
            self.jump_c.append(c_init_z)
            self.jump_cdot_bounds.append([0., 0., 0.])
            self.jump_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes jump
            self.jump_c.append(c_init_z + sin[k])
            self.jump_cdot_bounds.append([max_velocity, max_velocity, max_velocity])
            self.jump_f_bounds.append([0., 0., 0.])
        for k in range(0, 7):  # 6 nodes down
            self.jump_c.append(c_init_z)
            self.jump_cdot_bounds.append([0., 0., 0.])
            self.jump_f_bounds.append([max_force, max_force, max_force])
        #                                 ____
        #                                /    \
        #                               /      \
        # profile of the action _______/        \______
        # print(self.jump_c) # height contact jump
        # print(self.jump_cdot_bounds) # vel contact jump
        # print(self.jump_f_bounds) # f contacts bounds

        # NO STEP
        self.stance = []
        self.cdot_bounds = []
        self.f_bounds = []
        for k in range(0, nodes):
            self.stance.append([c_init_z])
            self.cdot_bounds.append([0., 0., 0.])
            self.f_bounds.append([max_force, max_force, max_force])

        # profile of the action ______________________
        # print(self.stance) # height contact jump
        # print(self.cdot_bounds) # vel contact jump
        # print(self.f_bounds) # f contacts bounds

        # STEP
        sin = 0.1 * np.sin(np.linspace(0, np.pi, 8))
        # left step cycle
        self.l_cycle = []
        self.l_cdot_bounds = []
        self.l_f_bounds = []
        for k in range(0, 2):  # 2 nodes down
            self.l_cycle.append(c_init_z)
            self.l_cdot_bounds.append([0., 0., 0.])
            self.l_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes step
            self.l_cycle.append(c_init_z + sin[k])
            self.l_cdot_bounds.append([max_velocity, max_velocity, max_velocity])
            self.l_f_bounds.append([0., 0., 0.])
        for k in range(0, 2):  # 2 nodes down
            self.l_cycle.append(c_init_z)
            self.l_cdot_bounds.append([0., 0., 0.])
            self.l_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes down (other step)
            self.l_cycle.append(c_init_z)
            self.l_cdot_bounds.append([0., 0., 0.])
            self.l_f_bounds.append([max_force, max_force, max_force])
        self.l_cycle.append(c_init_z)  # last node down
        self.l_cdot_bounds.append([0., 0., 0.])
        self.l_f_bounds.append([max_force, max_force, max_force])

        # right step cycle
        self.r_cycle = []
        self.r_cdot_bounds = []
        self.r_f_bounds = []
        for k in range(0, 2):  # 2 nodes down
            self.r_cycle.append(c_init_z)
            self.r_cdot_bounds.append([0., 0., 0.])
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes down (other step)
            self.r_cycle.append(c_init_z)
            self.r_cdot_bounds.append([0., 0., 0.])
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 2):  # 2 nodes down
            self.r_cycle.append(c_init_z)
            self.r_cdot_bounds.append([0., 0., 0.])
            self.r_f_bounds.append([max_force, max_force, max_force])
        for k in range(0, 8):  # 8 nodes step
            self.r_cycle.append(c_init_z + sin[k])
            self.r_cdot_bounds.append([max_velocity, max_velocity, max_velocity])
            self.r_f_bounds.append([0., 0., 0.])
        self.r_cycle.append(c_init_z)  # last node down
        self.r_cdot_bounds.append([0., 0., 0.])
        self.r_f_bounds.append([max_force, max_force, max_force])

        self.action = ""
        #                                       ___
        # L or F:                         ___  /   \
        #                                /   \/     \
        #                            ___/____/\      \_____
        # profile of the action _______/       \____
        print(self.l_cycle)  # height contact jump
        print(self.l_cdot_bounds)  # vel contact jump
        print(self.l_f_bounds)  # f contacts bounds

        print(self.r_cycle)  # height contact jump
        print(self.r_cdot_bounds)  # vel contact jump
        print(self.r_f_bounds)  # f contacts bounds


    def set(self, action):
        t = self.nodes - self.step_counter # this goes FROM nodes TO 0

        for k in range(max(t, 0), self.nodes + 1):
            ref_id = (k - t)%self.nodes
            # print(k)
            print(ref_id)

            if(ref_id == 0):
                self.action = action

            if action == "trot":
                for i in [0, 3]:
                    self.c_ref[i].assign(self.l_cycle[ref_id], nodes = k)
                    self.cdot[i].setBounds(-1.*np.array(self.l_cdot_bounds[ref_id]), np.array(self.l_cdot_bounds[ref_id]), nodes=k)
                    if k < self.nodes:
                        self.f[i].setBounds(-1.*np.array(self.l_f_bounds[ref_id]), np.array(self.l_f_bounds[ref_id]), nodes=k)
                for i in [1, 2]:
                    self.c_ref[i].assign(self.r_cycle[ref_id], nodes = k)
                    self.cdot[i].setBounds(-1.*np.array(self.r_cdot_bounds[ref_id]), np.array(self.r_cdot_bounds[ref_id]), nodes=k)
                    if k < self.nodes:
                        self.f[i].setBounds(-1.*np.array(self.r_f_bounds[ref_id]), np.array(self.r_f_bounds[ref_id]), nodes=k)

        self.step_counter += 1
if __name__ == '__main__':

    nc = 20
    number_of_legs = 2
    contact_model = 4
    max_force = 1000.
    max_velocity = 10.
    c_ref = dict()

    prb = Problem(nc)
    for i in range(0, nc):
        c_ref[i] = prb.createParameter("c_ref" + str(i), 1)

    c = dict()
    for i in range(0, nc):
        c[i] = prb.createStateVariable("c" + str(i), 3)  # Contact i position

    cdot = dict()
    for i in range(0, nc):
        cdot[i] = prb.createStateVariable("cdot" + str(i), 3)  # Contact i vel

    f = dict()
    for i in range(0, nc):
        f[i] = prb.createInputVariable("f" + str(i), 3)  # Contact i forces

    # for i in [0, 1, 2, 3]:
    #     print(c[i].getLowerBounds())
    #     print(c[i].getUpperBounds())
    #     print(cdot[i].getLowerBounds())
    #     print(cdot[i].getUpperBounds())
    #     print(f[i].getLowerBounds())
    #     print(f[i].getUpperBounds())
    #     print(c_ref[i].getValues())
    #     print(c_ref[i].getValues())
    #     print('=========================================')

    wpg = steps_phase(f, c, cdot, 0, c_ref, nc, number_of_legs, contact_model, max_force, max_velocity)
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')
    wpg.set('trot')

    # for i in [0, 1, 2, 3]:
    #     print(c[i].getLowerBounds())
    #     print(c[i].getUpperBounds())
    #     print(cdot[i].getLowerBounds())
    #     print(cdot[i].getUpperBounds())
    #     print(f[i].getLowerBounds())
    #     print(f[i].getUpperBounds())
    #     print(c_ref[i].getValues())
    #     print(c_ref[i].getValues())

#         elif action == "step":
#             for i in range(0, contact_model):
#                 self.c_ref[i].assign(self.l_cycle[ref_id], nodes = k)
#                 self.cdot[i].setBounds(-1.*np.array(self.l_cdot_bounds[ref_id]), np.array(self.l_cdot_bounds[ref_id]), nodes=k)
#                 if k < self.nodes:
#                     self.f[i].setBounds(-1.*np.array(self.l_f_bounds[ref_id]), np.array(self.l_f_bounds[ref_id]), nodes=k)
#             for i in range(contact_model, contact_model * number_of_legs):
#                 self.c_ref[i].assign(self.r_cycle[ref_id], nodes = k)
#                 self.cdot[i].setBounds(-1.*np.array(self.r_cdot_bounds[ref_id]), np.array(self.r_cdot_bounds[ref_id]), nodes=k)
#                 if k < self.nodes:
#                     self.f[i].setBounds(-1.*np.array(self.r_f_bounds[ref_id]), np.array(self.r_f_bounds[ref_id]), nodes=k)
#
#         elif action == "jump":
#             for i in range(0, len(c)):
#                 self.c_ref[i].assign(self.jump_c[ref_id], nodes = k)
#                 self.cdot[i].setBounds(-1. * np.array(self.jump_cdot_bounds[ref_id]), np.array(self.jump_cdot_bounds[ref_id]), nodes=k)
#                 if k < self.nodes:
#                     self.f[i].setBounds(-1. * np.array(self.jump_f_bounds[ref_id]), np.array(self.jump_f_bounds[ref_id]), nodes=k)
#
#         else:
#             for i in range(0, len(c)):
#                 self.c_ref[i].assign(self.stance[ref_id], nodes=k)
#                 self.cdot[i].setBounds(-1. * np.array(self.cdot_bounds[ref_id]), np.array(self.cdot_bounds[ref_id]), nodes=k)
#                 if k < self.nodes:
#                     self.f[i].setBounds(-1. * np.array(self.f_bounds[ref_id]), np.array(self.f_bounds[ref_id]), nodes=k)
#
#     self.step_counter += 1
