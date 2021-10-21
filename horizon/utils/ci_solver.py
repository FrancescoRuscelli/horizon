#!/usr/bin/env python
import xbot_interface.config_options as xbot_opt
import rospy
from cartesian_interface.pyci_all import *
from xbot_interface import xbot_interface as xbot
from xbot_interface import config_options as co
import numpy as np
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import yaml
import json
import pprint
from xbot_msgs.msg import JointState

class CartesianInterfaceSolver:
    '''
    inverse kinematics solver by CartesI/O
    input:
        model --> model of the robot
        ik_dt --> dt of the solver
        ctrl_points --> joint of the robot to control

    '''
    def __init__(self, model, robot, ik_dt):

        self.model = model
        self.robot = robot
        self.ik_dt = ik_dt

        # to visualize the robot in rviz
        self.rspub = pyci.RobotStatePublisher(self.model)

        self.ik_pb = self.__make_problem_desc_ik()
        self.log_path = '/tmp'
        self.ci = pyci.CartesianInterface.MakeInstance('OpenSot',
                                                       self.ik_pb,
                                                       self.model,
                                                       self.ik_dt,
                                                       log_path=self.log_path)

        self.com = self.ci.getTask('com')
        self.postural = self.ci.getTask('postural')

        self.l_arm = self.ci.getTask('l_ball_tip')
        self.r_arm = self.ci.getTask('r_ball_tip')

        self.l_sole = self.ci.getTask('l_sole')
        self.r_sole = self.ci.getTask('r_sole')

    def __make_problem_desc_ik(self):

        # cartesI/O stack:
        # math of task (mot)
        #  --> ctrl_points
        #  --> com
        #  --> postural

        ik_cfg = dict()

        ik_cfg['solver_options'] = {'regularization': 1e-4, 'back-end': 'osqp'}

        ik_cfg['stack'] = [['l_sole', 'r_sole'], ['com', 'l_ball_tip', 'r_ball_tip'], ['postural']]

        ik_cfg['constraints'] = ['JointLimits', 'VelocityLimits']

        ik_cfg['JointLimits'] = {'type': 'JointLimits'}
        ik_cfg['VelocityLimits'] = {'type': 'VelocityLimits'}

        ik_cfg['postural'] = {
            'name': 'postural',
            'type': 'Postural',
            'lambda': 0.,
        }

        ik_cfg['r_ball_tip'] = {
            'name': 'r_ball_tip',
            'type': 'Cartesian',
            'distal_link': 'r_ball_tip',
            'base_link': 'torso',#torso
            'lambda': 0.1, #0.1,
        }

        ik_cfg['l_ball_tip'] = {
            'name': 'l_ball_tip',
            'type': 'Cartesian',
            'distal_link': 'l_ball_tip',
            'base_link': 'torso', #torso
            'lambda': 0.1, #0.1,
        }

        ik_cfg['com'] = {
            'name': 'com',
            'type': 'Com',
            'lambda': 0.005,
        }

        ik_cfg['l_sole'] = {
                'name': 'l_sole',
                'type': 'Cartesian',
                'distal_link': 'l_sole',
                'lambda': 0.01
            }

        ik_cfg['r_sole'] = {
                'name': 'r_sole',
                'type': 'Cartesian',
                'distal_link': 'r_sole',
                'lambda': 0.01
            }

        ik_str = yaml.dump(ik_cfg)

        return ik_str

    def __ci_solve_integrate(self, t, sim):
        # integrate model
        if not self.ci.update(t, self.ik_dt):
            return False

        q = self.model.getJointPosition()
        qdot = self.model.getJointVelocity()
        qddot = self.model.getJointAcceleration()

        q += qdot * self.ik_dt + 0.5 * qddot * self.ik_dt ** 2
        qdot += qddot * self.ik_dt

        # print('is q_model (before) equal to q_robot?: {}'.format(np.isclose(q_robot, q_before[6:], 1e-7).all()))
        # print('is q_before equal to q_after?: {}'.format(np.isclose(q_before, q_after, 1e-7).all()))

        self.model.setJointPosition(q)
        self.model.setJointVelocity(qdot)
        self.model.update()

        if sim:
            self.robot.setPositionReference(q[6:])
            # print(self.robot.getJointPosition())
            # print(self.robot.getPositionReference())
            # print('is q_robot equal to q_commanded?: {}'.format(np.isclose(self.robot.getJointPosition(), self.robot.getPositionReference(), 1e-7).all()))
            self.robot.move()

        return True

    def reach(self, task, goal, duration, sim=0):

        ## Cartesian part
        q = np.empty(shape=[self.model.getJointNum(), 0])

        task.setActivationState(pyci.ActivationState.Enabled)

        time_from_reaching = 0.
        unable_to_solve = 0
        ci_time = 0.0
        initialize_trj = False

        CONVERGENCE_TIME = 5.
        UNABLE_TO_SOLVE_MAX = 5

        task.setPoseTarget(goal, duration)

        while task.getTaskState() == pyci.State.Reaching or time_from_reaching <= CONVERGENCE_TIME:

            q = np.hstack((q, self.model.getJointPosition().reshape(self.model.getJointNum(), 1)))

            if not self.__ci_solve_integrate(ci_time, sim):
                print('Unable to solve!!!')
                unable_to_solve += 1
                print(unable_to_solve)
                # break
                if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                    print("Maximum number of unable_to_solve reached: ")
                    print(unable_to_solve)
                    return q, False
            else:
                unable_to_solve = 0

            ci_time += self.ik_dt

            if task.getTaskState() == pyci.State.Online:
                if not initialize_trj:
                    initialize_trj = True

                time_from_reaching += self.ik_dt

            self.rspub.publishTransforms('ci')


    def getTasks(self):
        return self.l_sole, self.r_sole, self.l_arm, self.r_arm, self.com

    def update(self, ci_time, sim=0):

        q = np.empty(shape=[self.model.getJointNum(), 0])
        unable_to_solve = 0
        UNABLE_TO_SOLVE_MAX = 5
        if not self.__ci_solve_integrate(ci_time, sim):
            print('Unable to solve!!!')
            unable_to_solve += 1
            print(unable_to_solve)
            # break
            if unable_to_solve >= UNABLE_TO_SOLVE_MAX:
                print("Maximum number of unable_to_solve reached: ")
                print(unable_to_solve)
                return q, False
        else:
            unable_to_solve = 0

        self.rspub.publishTransforms('ci')


if __name__ == '__main__':

    np.set_printoptions(precision=3, suppress=True)

    rospy.init_node('ci_solver_try')
    roscpp_init('ci_solver_try', [])

    opt = xbot_opt.ConfigOptions()

    urdf = rospy.get_param('/xbotcore/robot_description')
    srdf = rospy.get_param('/xbotcore/robot_description_semantic')

    opt = co.ConfigOptions()
    opt.set_urdf(urdf)
    opt.set_srdf(srdf)
    opt.generate_jidmap()
    opt.set_bool_parameter('is_model_floating_base', True)
    opt.set_string_parameter('model_type', 'RBDL')
    opt.set_string_parameter('framework', 'ROS')
    model = xbot.ModelInterface(opt)
    robot = xbot.RobotInterface(opt)


    robot.sense()
    initial_joint_state = rospy.wait_for_message('/xbotcore/joint_states', JointState)



    # notice how if I only do this, there is a little gulp of the robot
    model.syncFrom(robot)

    # instead of syncing from robot, give the model the POSITION REFERENCE after the homing of the robot
    pos_ref = [0,0,0,0,0,0]
    pos_ref.extend(list(initial_joint_state.position_reference))
    model.setJointPosition(pos_ref)

    model.update()
    world_gazebo = model.getPose('l_sole')

    print('r_sole:', model.getPose('r_sole'))
    print('l_sole:', world_gazebo)
    world_gazebo.translation[1] += model.getPose('r_sole').translation[1]

    print('world_gazebo:', world_gazebo)
    w_T_fb = model.getFloatingBasePose()
    print('w_T_fb_before:', w_T_fb)
    model.setFloatingBasePose(w_T_fb * world_gazebo.inverse())

    model.update()

    print('w_T_fb_after:', model.getFloatingBasePose())

    ci_solver = CartesianInterfaceSolver(model=model, robot=robot, ik_dt=0.01)
    print('Created cartesian interface.')

    l_sole_task, r_sole_task, l_arm_task, r_arm_task, com_task = ci_solver.getTasks()
    #
    x_l_foot = model.getPose(l_sole_task.getName()).translation[0]
    x_r_foot = model.getPose(r_sole_task.getName()).translation[0]

    duration = 2

    com_initial = model.getCOM()
    print('initial_com:', model.getCOM())
    goal_com = Affine3()
    goal_com.translation = com_initial
    goal_com.translation[0] = x_r_foot
    print('commanded_com:', goal_com)

    ci_solver.reach(com_task, goal_com, duration, sim=1)
    print('goal_com:', model.getCOM())

    # l_arm_initial = model.getPose(l_arm_task.getName())
    # print('l_arm_initial:', l_arm_initial)


    # goal_l_arm = Affine3
    # goal_l_arm = l_arm_initial
    # goal_l_arm.translation[2] += 0.01
    # print('commanded_l_arm:', goal_l_arm)
    # ci_solver.reach(l_arm_task, goal_l_arm, duration, sim=0)
    # print('l_arm_goal:', model.getPose(l_arm_task.getName()))
    # for i in range(100):
    #
    #     traj.translation[2] = traj.translation[2] + 0.001
    #
    #     print(traj.translation)
    #     ci_solver.sendTrajectory(ci_solver[0], traj)
    #
    #     rospy.sleep(0.01)


