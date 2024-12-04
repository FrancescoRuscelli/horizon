#!/usr/bin/env python3

import argparse

def str2bool(v):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")

if __name__ == '__main__':

  spot_actions = ('wheelie', 'jump_up', 'jump_forward', 'jump_on_wall', 'leap', 'jump_twist', 'walk')
  spot_solvers = ('ipopt', 'ilqr', 'gnsqp')

  parser = argparse.ArgumentParser(description='motion planning for the BostonDynamics quadruped robot!')
  parser.add_argument('--action', '-a', help='choose which action spot will perform', choices=spot_actions, default=spot_actions[1])
  parser.add_argument('--solver', '-s', help='choose which solver will be used', choices=spot_solvers, default=spot_solvers[0])
  parser.add_argument('--replay', '-r', help='visualize the robot trajectory in rviz', action='store_true', default=False)
  parser.add_argument("--codegen", '-c', type=str2bool, nargs='?', const=True, default=False, help="generate c++ code for faster solving")
  parser.add_argument("--warmstart", '-w', type=str2bool, nargs='?', const=True, default=False, help="save solutions to mat file")
  parser.add_argument("--plot", '-p', type=str2bool, nargs='?', const=True, default=True, help="plot solutions")

  args = parser.parse_args()

  action = args.action

  if action == 'walk':
    from horizon.examples import spot_walk
    spot_walk.main(args)

  else:
    from horizon.examples import spot_motions
    spot_motions.main(args)
