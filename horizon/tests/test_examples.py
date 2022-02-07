import argparse
import os

def testSpot():

    spot_actions = ('wheelie', 'jump_up', 'jump_forward', 'jump_on_wall', 'leap', 'jump_twist', 'walk')
    spot_solvers = ('ipopt', 'ilqr', 'gnsqp')

    try:

        for action in spot_actions:
            arg_combination = '--action ' + action
            if action == 'walk':
                print(arg_combination)
                # os.system('python3 -m horizon.examples.spot --plot False ' + arg_combination)
            else:
                for solver in spot_solvers:
                    arg_combination = '--action ' + action + ' --solver ' + solver
                    print(arg_combination)
                    # os.system('python3 -m horizon.examples.spot --plot False ' + arg_combination)

        return True
    except:
            return False

if __name__ == '__main__':
    a = testSpot()
    print(a)