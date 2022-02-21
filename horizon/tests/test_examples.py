import argparse
import os
import unittest
import subprocess

class TestExamples(unittest.TestCase):

    def runExample(self, process_command):

        try:
            print(process_command)
            subprocess.run(process_command, check=True)
        except:
            self.fail()

    def testSpot(self):
        robot = 'spot'
        actions = ('wheelie', 'jump_up', 'jump_forward', 'jump_on_wall', 'leap', 'jump_twist', 'walk')
        spot_solvers = ('ipopt', 'ilqr', 'gnsqp')


        for action in actions:

            process_command = ['python3', '-m', 'horizon.examples.' + robot]
            process_command.extend(['--plot', 'False', '--action', action])

            if action == 'walk':
                self.runExample(process_command)
            else:
                for solver in spot_solvers:
                    solver_command = ['--solver', solver]
                    self.runExample(process_command + solver_command)
        
        return True

    def testRopedRobot(self):
        robot = 'roped_robot'
        actions = ('swing', 'free_fall', 'hang', 'rappel')

        for action in actions:
            process_command = ['python3', '-m', 'horizon.examples.' + robot]
            arg_combination = ['--plot', 'False', '--action', action]

            process_command.extend(arg_combination)
            self.runExample(process_command)
        
        return True


    def testQuadruped(self):
        robot = 'quadruped'
        process_command = ['python3', '-m', 'horizon.examples.' + robot]
        process_command.extend(['--plot', 'False'])

        self.runExample(process_command)

        return True

    def testCartPole(self):
        robot = 'cart_pole'
        inputs = ['torque', 'acceleration']


        for input_i in inputs:
            process_command = ['python3', '-m', 'horizon.examples.' + robot]
            process_command.extend(['--plot', 'False', '--input', input_i])
            options = ['--minimize_t', '--parametric']
            # add all options and remove one by one to test everything
            for i in range(len(options) + 1):
                self.runExample(process_command + options)
                options = options[:-1]

        return True


if __name__ == '__main__':

    unittest.main()
    # test = TestExamples()

    # test.assertTrue(test.testSpot())
    # print(test.testCartPole())
    # test.testQuadruped()
    # test.testRopedRobot()