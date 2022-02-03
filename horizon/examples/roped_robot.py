import time

from horizon import problem
from horizon.utils import utils, kin_dyn, resampler_trajectory, plotter, mat_storer
from horizon.transcriptions import integrators
from horizon.transcriptions.transcriptor import Transcriptor
from horizon.ros.replay_trajectory import *
from horizon.solvers import solver
import matplotlib.pyplot as plt
import os, rospkg, argparse
from scipy.io import loadmat
from itertools import filterfalse

def str2bool(v):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

roped_robot_actions = ('swing', 'free_fall', 'hang', 'rappel')

parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
parser.add_argument('--replay', help='visualize the robot trajectory in rviz', action='store_true')
parser.add_argument('--action', '-a', help='choose which action spot will perform', choices=roped_robot_actions, default=roped_robot_actions[1])

args = parser.parse_args()

action = args.action

if action == 'rappel':

  import roped_robot_rappel
else:
  import roped_robot_sim
