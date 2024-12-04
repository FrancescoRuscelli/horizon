#!/usr/bin/env python3

import argparse

def str2bool(v):
  #susendberg's function
  return v.lower() in ("yes", "true", "t", "1")


def main(args):

  action = args.action

  if action == 'rappel':
    from horizon.examples import roped_robot_rappel
    roped_robot_rappel.main(args)

  else:
    from horizon.examples import roped_robot_sim
    roped_robot_sim.main(args)


if __name__ == '__main__':

  roped_robot_actions = ('swing', 'free_fall', 'hang', 'rappel')

  parser = argparse.ArgumentParser(description='cart-pole problem: moving the cart so that the pole reaches the upright position')
  parser.add_argument('--replay', help='visualize the robot trajectory in rviz', action='store_true')
  parser.add_argument('--action', '-a', help='choose which action spot will perform', choices=roped_robot_actions, default=roped_robot_actions[1])
  parser.add_argument("--plot", '-p', type=str2bool, nargs='?', const=True, default=True, help="plot solutions")
  parser.add_argument("--warmstart", '-w', type=str2bool, nargs='?', const=True, default=False,
                      help="save solutions to mat file")

  args = parser.parse_args()
  main(args)