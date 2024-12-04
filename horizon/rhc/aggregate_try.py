import os
import numpy as np
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils.actionManager import ActionManager

# set up problem
ns = 50
tf = 2.0  # 10s
dt = tf / ns

# set up solver
solver_type = 'ilqr'

# set up model
path_to_examples = os.path.dirname('../examples/')
urdffile = os.path.join(path_to_examples, 'urdf', 'spot.urdf')
urdf = open(urdffile, 'r').read()

contacts = ['lf_foot', 'rf_foot', 'lh_foot', 'rh_foot']

base_init = np.array([0.0, 0.0, 0.505, 0.0, 0.0, 0.0, 1.0])
q_init = {'lf_haa_joint': 0.0,
          'lf_hfe_joint': 0.9,
          'lf_kfe_joint': -1.52,

          'lh_haa_joint': 0.0,
          'lh_hfe_joint': 0.9,
          'lh_kfe_joint': -1.52,

          'rf_haa_joint': 0.0,
          'rf_hfe_joint': 0.9,
          'rf_kfe_joint': -1.52,

          'rh_haa_joint': 0.0,
          'rh_hfe_joint': 0.9,
          'rh_kfe_joint': -1.52}

problem_opts = {'ns': ns, 'tf': tf, 'dt': dt}
model_description = 'whole_body'

# todo: wrong way of adding the contacts contacts=['lf_foot']
ti = TaskInterface(urdf, q_init, base_init, problem_opts, model_description)

ti.setTaskFromYaml('aggregate.yaml')

exit()

f0 = np.array([0, 0, 55])
contact1 = ti.getTask('joint_regularization')
contact1.setRef(1, f0)
contact1.setRef(2, f0)
contact1.setRef(3, f0)
contact1.setRef(4, f0)
