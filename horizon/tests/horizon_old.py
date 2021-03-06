# /usr/bin/env python3
import casadi as cs
import numpy as np
import warnings
from collections import OrderedDict
import math
import time
import pprint

def ordered_dict_prepend(dct, key, value, dict_setitem=dict.__setitem__):
    root = dct._OrderedDict__root
    first = root[1]

    if key in dct:
        link = dct._OrderedDict__map[key]
        link_prev, link_next, _ = link
        link_prev[1] = link_next
        link_next[0] = link_prev
        link[0] = root
        link[1] = first
        root[1] = first[0] = link
    else:
        root[1] = first[0] = dct._OrderedDict__map[key] = [root, first, key]
        dict_setitem(dct, key, value)

def interpolator(traj_old, step_i, step_f, step_height, time, t_i, t_f, freq):
    # todo do something with the traj_old
    traj = dict()
    # print('traj_old', traj_old.shape)
    traj_len = np.ceil(float(freq) * float(t_f - t_i))
    # print('traj_len', traj_len)
    traj_len_before = np.ceil(float(freq) * float(t_i))
    # print('traj_len_before', traj_len_before)
    traj_len_after = np.ceil(float(freq) * float(time-t_f)) + 1 # todo for now is N+1 so traj_len_after lasts 1 node more
    # print('traj_len_after', traj_len_after)

    t = np.linspace(0, 1, np.ceil(traj_len))
    dt = 1. / float(freq)

    traj['x'] = np.full(traj_len_before, step_i[0])
    traj['y'] = np.full(traj_len_before, step_i[1])
    traj['z'] = np.full(traj_len_before, 0.)

    traj['dx'] = np.full(traj_len_before, 0.)
    traj['dy'] = np.full(traj_len_before, 0.)
    traj['dz'] = np.full(traj_len_before, 0.)

    traj['ddx'] = np.full(traj_len_before, 0.)
    traj['ddy'] = np.full(traj_len_before, 0.)
    traj['ddz'] = np.full(traj_len_before, 0.)

    traj['x'] = np.append(traj['x'], (step_i[0] + (((6. * t - 15.) * t + 10.) * t ** 3.) * (step_f[0] - step_i[0])))  # on the x
    traj['y'] = np.append(traj['y'], (step_i[1] + (((6. * t - 15.) * t + 10.) * t ** 3.) * (step_f[1] - step_i[1]))) # on the y
    traj['z'] = np.append(traj['z'], (64. * t ** 3. * (1. - t) ** 3.) * step_height) # on the z

    traj['x'] = np.append(traj['x'], np.full(traj_len_after, step_f[0]))
    traj['y'] = np.append(traj['y'], np.full(traj_len_after, step_f[1]))
    traj['z'] = np.append(traj['z'], np.full(traj_len_after, 0.))

    # compute velocity # todo remember this, derivative of function and then (t_f - t_i) to make it consistent with the parametrization
    traj['dx'] = np.append(traj['dx'], 30. * (t - 1.) ** 2. * t ** 2. * (step_f[0] - step_i[0])) / (t_f - t_i)
    traj['dy'] = np.append(traj['dy'], 30. * (t - 1.) ** 2. * t ** 2. * (step_f[1] - step_i[1])) / (t_f - t_i)
    traj['dz'] = np.append(traj['dz'], step_height * (t - 1.) ** 2. * (t ** 2.0 * (192. - 192. * t) - 192. * t ** 3.)) / (t_f - t_i)

    traj['dx'] = np.append(traj['dx'], np.full(traj_len_after, 0.))
    traj['dy'] = np.append(traj['dy'], np.full(traj_len_after, 0.))
    traj['dz'] = np.append(traj['dz'], np.full(traj_len_after, 0.))

    # compute acceleration
    # traj['ddx'] = np.append(traj['ddx'], 60 * t * (2 * t ** 2 - 3. * t + 1) * (step_f[0] - step_i[0]))
    # traj['ddy'] = np.append(traj['ddy'], 60 * t * (2 * t ** 2 - 3. * t + 1) * (step_f[1] - step_i[1]))
    # traj['ddz'] = np.append(traj['ddz'], step_height * (384.0 * t ** 1.0 - 2304.0 * t ** 2.0 + 3840.0 * t ** 3.0 - 1920.0 * t ** 4.0))
    #
    # traj['ddx'] = np.append(traj['ddx'], np.full(traj_len_after, 0.))
    # traj['ddy'] = np.append(traj['ddy'], np.full(traj_len_after, 0.))
    # traj['ddz'] = np.append(traj['ddz'], np.full(traj_len_after, 0.))

    # small hack for t filling
    t = np.linspace(0, 1, len(traj['x']))

    return t, traj


def casadi_sum(x, axis=None, out=None):
    assert out is None
    if axis == 0:
        return cs.sum1(x)
    elif axis == 1:
        return cs.sum2(x)
    elif axis is None:
        return cs.sum1(cs.sum2(x))
    else:
        raise Exception("Invalid argument for sum")

def RK4(M, L, x, u, xdot, dt):
    """RK4 Runge-Kutta 4 integrator
    TODO: PUT IT IN ANOTHER CLASS
    Input:
        L: objective fuction to integrate
        M: RK steps
        T: final time
        N: numbr of shooting nodes:
        x: state varibales
        u: controls
    """

    f = cs.Function('f', [x, u], [xdot, L])

    X0 = cs.SX.sym('X0', x.size()[0])
    U = cs.SX.sym('U', u.size()[0])
    X = X0
    Q = 0

    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + dt / 2 * k1, U)
        k3, k3_q = f(X + dt / 2 * k2, U)
        k4, k4_q = f(X + dt * k3, U)
        X = X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        Q = Q + dt / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

    return cs.Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])


class Problem:

    def __init__(self, N, crash_if_suboptimal=False):

        self.crash_if_suboptimal = crash_if_suboptimal
        self.N = N+1 # todo here decide if N or N+1
        self.x = list()
        self.u = list()
        self.lbw = list()
        self.ubw = list()

        self.sym_c = cs.SX

        self.ct = Constraint(self.N)

        self.var_opt_prev = OrderedDict()
        self.var_opt = OrderedDict()
        self.fun_opt = dict()

        self.var_dim = dict()

        self.j_dict = dict()

        # all the variables in the problem for all the nodes
        self.state_var = list()
        self.w0_list = list()

        self.w0 = np.array([])

        self.j = 0

        # symbolic variables of the problems (without past variables) and their dimension
        self.prb_vars = OrderedDict()

    def setVariable(self, name, var_dim, k_node=[]):

        if k_node:
            self.var_opt[name + str(k_node)] = self.sym_c.sym(name + str(k_node), var_dim)
            self.var_dim[name + str(k_node)] = var_dim
        else:
            self.var_opt[name] = self.sym_c.sym(name, var_dim)
            self.var_dim[name] = var_dim

        self.ct.update(self.var_opt)

    def setFunction(self, name, f):

        self.fun_opt[name] = f

    def getVariable(self, k=[]):
        if k:
            for elem in self.var_opt:
                self.var_opt[elem + '-' + str(k)] = list()
        else:
            pass

        return self.var_opt

    def getFunction(self):

        return self.fun_opt

    def buildProblem(self):

        for name, var in self.var_opt.items():
            if name.find('-') == -1:
                self.prb_vars[name] = var.shape

        for k in range(self.N): # todo decide if N or N+1

            # print('----- node', k, '-----')

            state_var_k = list()
            w0_k = dict()

            self.updateVariables(k)

            for var in self.var_opt:
                if var.find('-') == -1:
                    if k != self.N-1:
                        state_var_k.append(dict(name=var, var=self.var_opt[var], lbw=[-cs.inf] * self.var_opt[var].shape[0], ubw=[cs.inf] * self.var_opt[var].shape[0]))
                        w0_k.update({var: np.zeros((self.var_opt[var].shape[0]))})
                    else: # todo very hacky remove last input
                        if var == 'u':
                            pass
                        else:
                            state_var_k.append(dict(name=var, var=self.var_opt[var], lbw=[-cs.inf] * self.var_opt[var].shape[0], ubw=[cs.inf] * self.var_opt[var].shape[0]))
                            w0_k.update({var: np.zeros((self.var_opt[var].shape[0]))})
                            # w0_k.append(self.w0, np.zeros((self.var_opt[var].shape[0])))


            self.state_var.append(state_var_k)
            self.w0_list.append(w0_k)

            # todo remove last input
            # todo make possible to define what to include in state variable and what not

            # print('adding constraint functions:')
            # add new constraint with changed input
            self.ct.update(self.var_opt)
            for constraint in self.ct.g_dict:
                # print('ADDING CONSTRAINT:', constraint)
                # add constraint only if in the specified nodes
                if any(isinstance(el, list) for el in self.ct.g_dict[constraint]['nodes']):
                    for chunk in self.ct.g_dict[constraint]['nodes']:
                        if k in range(chunk[0], chunk[1]):
                            self.ct.addConstraint(constraint)
                else:
                    if k in range(self.ct.g_dict[constraint]['nodes'][0], self.ct.g_dict[constraint]['nodes'][1]):
                        self.ct.addConstraint(constraint)
            # print('===============================')
            # add new cost function

            for cost_fun in self.j_dict:
                if any(isinstance(el, list) for el in self.j_dict[cost_fun]['nodes']):
                    for chunk in self.j_dict[cost_fun]['nodes']:
                        if k in range(chunk[0], chunk[1]):
                            self.addCostFunction(cost_fun)
                else:
                    if k in range(self.j_dict[cost_fun]['nodes'][0], self.j_dict[cost_fun]['nodes'][1]):
                        self.addCostFunction(cost_fun)

        print(self.j)
        # todo this is useless here! to place in solve problem, not in build problem
        self.ct.addConstraintBounds()

        self.addInitialGuess()

        # print([item['var'] for sublist in self.state_var for item in sublist])
        self.all_w = cs.vertcat(*[item['var'] for sublist in self.state_var for item in sublist])

        self.lbw = [item for sublist in [item['lbw'] for sublist in self.state_var for item in sublist] for item in sublist]
        self.ubw = [item for sublist in [item['ubw'] for sublist in self.state_var for item in sublist] for item in sublist]

        # w = cs.vertcat(*X, *U)
        # todo refactor this orrible stuff
        # w = [None] * (len(X) + len(U))
        # w[0::2] = X
        # w[1::2] = U
        print('getConstraints', self.ct.getConstraints())
        self.all_g = self.ct.getConstraints()

        J = self.j

        prob = {'f': J, 'x': self.all_w, 'g': self.all_g}
        self.solver = cs.nlpsol('solver', 'ipopt', prob)#,
                           # {'ipopt': {'linear_solver': 'ma27', 'tol': 1e-4, 'print_level': 3, 'sb': 'yes'},
                           #  'print_time': 0})  # 'acceptable_tol': 1e-4(ma57) 'constr_viol_tol':1e-3

        return self.all_w, self.all_g

    def solveProblem(self):

        print('================')
        print('len w:', self.all_w.shape)
        print('len lbw:', len(self.lbw))
        print('len ubw:', len(self.ubw))
        print('len w0:', len(self.w0))

        print('len g:', self.all_g.shape)
        print('len lbg:', len(self.ct.lbg))
        print('len ubg:', len(self.ct.ubg))

        print('================')
        print('w:', self.all_w)
        print('lbw:', self.lbw)
        print('ubw:', self.ubw)
        print('g:', self.all_g)
        print('lbg:', self.ct.lbg)
        print('ubg:', self.ct.ubg)
        print('j:', self.j)

        # Solve the NLP
        sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.ct.lbg, ubg=self.ct.ubg)

        if self.crash_if_suboptimal:
            if not self.solver.stats()['success']:
                raise Exception('Optimal solution NOT found.')

        w_opt = sol['x'].full().flatten()

        return w_opt

    def updateVariables(self, k):

        if k > 0:
            self.var_opt_prev[k - 1] = dict()

        for elem in self.var_opt:
            # search for past variables. If found, put it in 'self.var_opt_prev'
            # var_opt_prev keep track of all the past variables
            if elem.find('-') != -1:
                if k > 0:
                    name = elem[:elem.index('-')]
                    self.var_opt_prev[k - 1].update({name : self.var_opt[name]})

        for elem in self.var_opt:
            if elem.find('-') == -1:
                self.var_opt[elem] = self.sym_c.sym(elem + '_' + str(k), self.var_dim[elem])
            else:
                k_prev = int(elem[elem.index('-'):])
                name = elem[:elem.index('-')]
                if k+k_prev in self.var_opt_prev:
                    self.var_opt[elem] = self.var_opt_prev[k+k_prev][name]

    def setStateBounds(self, lbw, ubw, nodes):

        self.lbw[nodes[0]:nodes[1]] = [lbw] * (nodes[1] - nodes[0])
        self.ubw[nodes[0]:nodes[1]] = [ubw] * (nodes[1] - nodes[0])

    def setStateBoundsFromName(self, name, ubw, lbw, nodes=None):

        if nodes is None:
            nodes = [0, self.N]

        # print('state variable {} at node {}: lower bounds: {}'.format(name, nodes, lbw))

        if isinstance(nodes, int):
            for var_dict in self.state_var[nodes]:
                if var_dict['name'] == name:
                    var_dict['lbw'] = lbw
                    var_dict['ubw'] = ubw
        elif isinstance(nodes, list):
            for node in range(nodes[0], nodes[1]):
                for var_dict in self.state_var[node]:
                    if var_dict['name'] == name:
                        var_dict['lbw'] = lbw
                        var_dict['ubw'] = ubw

        self.lbw = [item for sublist in [item['lbw'] for sublist in self.state_var for item in sublist] for item in sublist]
        self.ubw = [item for sublist in [item['ubw'] for sublist in self.state_var for item in sublist] for item in sublist]

    def setCostFunction(self, name, j, nodes=None):

        # TODO check if variable exists in nodes (BUG: if added from node 0 the variable x-1, it does NOT give error)
        used_var = dict()
        # select from all variables only the variables used by the added constrain function
        for name_var, var in list(self.var_opt.items()):
            if cs.depends_on(j, var):
                used_var[name_var] = var

        f = cs.Function(name, list(used_var.values()), [j])

        if not nodes:
            nodes = [0, self.N]

        if any(isinstance(el, list) for el in nodes):
            n_chunks = len(nodes)
        else:
            n_chunks = 1

        cost_function = dict(cost_function=f,
                             var=list(used_var.keys()),
                             nodes=nodes)

        ## determine if there are lists in list of nodes
        if n_chunks > 1:
            if len(nodes) > len(set([item for sublist in nodes for item in sublist])):
                raise Exception('Intersecting lists of nodes.')

        ## if nodes of constraints is outside the range of nodes in the problem, trim
        if n_chunks > 1:
            if max(nodes)[0] > self.N:
                warnings.warn('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}). Removing.'.format([max(nodes)[0], max(nodes)[1]],  self.N))
                nodes.remove(max(nodes))

            if max(nodes)[1] > self.N:
                warnings.warn('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}). Trimming.'.format(max(nodes)[1], self.N))
                max(nodes)[1] = self.N
        else:
            if nodes[0] > self.N:
                raise Exception('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}).'.format(nodes[0], self.N))

            if nodes[1] > self.N:
                warnings.warn('WARNING: lists of constraints(max: {0}) nodes outside the problem nodes (max: {1}). Trimming.'.format(nodes[1], self.N))
                nodes[1] = self.N

        self.j_dict[name] = cost_function

    def addCostFunction(self, name):

        f = self.j_dict[name]['cost_function']
        j = f(*[self.var_opt[x] for x in self.j_dict[name]['var']])
        # print('j:', j)
        self.j = self.j + j

    def showVariables(self):

        k = 0
        for state in self.state_var:
            k += 1
            print('======node {} ======='.format(k))
            for elem in state:
                print(elem)

    def getOptimizedVariables(self, w_opt):

        # #todo hacky hack to split x into p, v, a

        prb_vars_ordered = OrderedDict(self.prb_vars)


        if 'x' in prb_vars_ordered:
            del prb_vars_ordered['x']
            #
            ordered_dict_prepend(prb_vars_ordered, 'a', (2, 1))
            ordered_dict_prepend(prb_vars_ordered, 'v', (2, 1))
            ordered_dict_prepend(prb_vars_ordered, 'p', (2, 1))

            # with python >3.2
            # prb_vars_ordered.update(p=(2, 1), v=(2, 1), a=(2, 1))
            # prb_vars_ordered.move_to_end('a', last=False)
            # prb_vars_ordered.move_to_end('v', last=False)
            # prb_vars_ordered.move_to_end('p', last=False)

        # todo careful about ordering
        # filling arrays with zeros
        num_var = sum([var[0] for name, var in prb_vars_ordered.items()])
        opt_values = dict()
        for name, var in prb_vars_ordered.items():
            if name == 'u':
                opt_values[name] = np.zeros([var[0], self.N-1])
            else:
                opt_values[name] = np.zeros([var[0], self.N])

        # fill first N-1 values
        for k in range(self.N-1):
            sol_k = w_opt[num_var * k:(num_var * k + num_var)]
            j = 0

            for name, var in prb_vars_ordered.items():
                for dim_i in range(opt_values[name].shape[0]):
                    opt_values[name][dim_i, k] = sol_k[j]
                    j += 1

        # fill last value without u
        del prb_vars_ordered['u']
        num_var_last = sum([var[0] for name, var in prb_vars_ordered.items()])
        sol_N = w_opt[-num_var_last:]

        j = 0
        for name, var in prb_vars_ordered.items():
            for dim_i in range(opt_values[name].shape[0]):
                opt_values[name][dim_i, self.N-1] = sol_N[j]
                j += 1

        return opt_values

    def setInitialGuess(self, name, nodes, vals):
        if isinstance(nodes, list):
            for node in range(nodes[0], nodes[1]):
                self.w0_list[node][name] = vals
        elif isinstance(nodes, int):
            self.w0_list[nodes][name] = vals

        # print(self.w0_list)
        self.addInitialGuess()

    def addInitialGuess(self):

        self.w0 = np.array([])
        for node_k in self.w0_list:
            # concatenate all the var in the node
            to_append = np.concatenate([np.array(val) for name, val in node_k.items()])
            # concatenate the node with all the others
            self.w0 = np.concatenate([self.w0, to_append])

class Constraint:

    def __init__(self, N):

        self.N = N
        self.g = list()
        self.lbg = list()
        self.ubg = list()

        self.g_dict = dict()
        self.j_dict = dict()

    def setConstraintFunction(self, name, g, nodes=None, bounds=None):

        used_var = dict()
        # select from all variables only the variables used by the added constrain function
        for name_var, var in list(self.var_opt.items()):
            if cs.depends_on(g, var):
                used_var[name_var] = var
                # check if variable exists in the full range of nodes
                if name_var.find('-') != -1:  # get from 'nodes' the first constrained node
                    if any(isinstance(el, list) for el in nodes): #todo problem if its list of list or just a list duplicated code
                        if min(nodes)[0] - int(name_var[name_var.index('-') + len('-'):]) < 0:
                            raise Exception('Failed to add constraint: variable', name_var,
                                            'can only be added from node n:',
                                            int(name_var[name_var.index('-') + len('-'):]))
                    else:
                        if nodes[0] - int(name_var[name_var.index('-') + len('-'):]) < 0:
                            raise Exception('Failed to add constraint: variable', name_var,
                                            'can only be added from node n:',
                                            int(name_var[name_var.index('-') + len('-'):]))

        print('used var in constraint:', used_var)
        # create function and add it to dictionary of constraint function
        f = cs.Function(name, list(used_var.values()), [g])

        if not nodes:
            nodes = [0, self.N]

        if any(isinstance(el, list) for el in nodes):
            n_chunks = len(nodes)
        else:
            n_chunks = 1

        constraint = dict(constraint=f,
                          var=list(used_var.keys()),
                          nodes=nodes,
                          bounds=dict())

        ## determine if there are lists in list of nodes
        if n_chunks > 1:
            if len(nodes) > len(set([item for sublist in nodes for item in sublist])):
                raise Exception('Intersecting lists of nodes.')

        ## if nodes of constraints is outside the range of nodes in the problem, trim
        if n_chunks > 1:
            if max(nodes)[0] > self.N:
                warnings.warn('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}). Removing.'.format([max(nodes)[0], max(nodes)[1]],  self.N))
                nodes.remove(max(nodes))

            if max(nodes)[1] > self.N:
                warnings.warn('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}). Trimming.'.format(max(nodes)[1], self.N))
                max(nodes)[1] = self.N
        else:
            if nodes[0] > self.N:
                raise Exception('WARNING: lists of constraints nodes (max: {0}) outside the problem nodes (max: {1}).'.format(nodes[0], self.N))

            if nodes[1] > self.N:
                warnings.warn('WARNING: lists of constraints(max: {0}) nodes outside the problem nodes (max: {1}). Trimming.'.format(nodes[1], self.N))
                nodes[1] = self.N

        ## check if list of nodes make sense

        # if not bounds, set all bounds as -inf, inf
        if not bounds:
            if n_chunks > 1:
                for chunk in nodes:
                    for elem in range(chunk[0], chunk[1]):
                        constraint['bounds'][elem] = dict(lbg=[-cs.inf] * g.shape[0], ubg=[cs.inf] * g.shape[0])
            else:
                for elem in range(nodes[0], nodes[1]):
                    constraint['bounds'][elem] = dict(lbg=[-cs.inf] * g.shape[0], ubg=[cs.inf] * g.shape[0])

        # if it's a dict, propagate everything
        if isinstance(bounds, dict):

            if 'nodes' in bounds and ('lbg' not in bounds and 'ubg' not in bounds):
                raise Exception('Required elements "lbg" and "ubg" are not inserted.')

            if 'nodes' not in bounds:
                if n_chunks > 1:
                    for chunk in nodes:
                        for elem in range(chunk[0], chunk[1]):
                            constraint['bounds'][elem] = dict()
                else:
                    for elem in range(nodes[0], nodes[1]):
                        constraint['bounds'][elem] = dict()
            else:
                if isinstance(bounds['nodes'], int):
                    constraint['bounds'][bounds['nodes']] = dict()
                elif isinstance(bounds['nodes'], list):
                    for el in range(bounds['nodes'][0], bounds['nodes'][1]):
                        constraint['bounds'][el] = dict()

            if 'lbg' not in bounds:
                for elem in constraint['bounds']:
                    constraint['bounds'][elem]['lbg'] = [-cs.inf] * g.shape[0]

            if 'ubg' not in bounds:
                for elem in constraint['bounds']:
                    constraint['bounds'][elem]['ubg'] = [cs.inf] * g.shape[0]

            if 'lbg' in bounds and len(bounds['lbg']) != g.shape[0]:
                raise Exception('Dimension of lower bounds {0} does not coincide with the constraint dimension {1}'.format(len(bounds['lbg']), g.shape[0]))
            if 'ubg' in bounds and len(bounds['ubg']) != g.shape[0]:
                raise Exception('Dimension of upper bounds {0} does not coincide with the constraint dimension (1}'.format(len(bounds['ubg']), g.shape[0]))

            for cnsrt_elem in constraint['bounds']:
                if 'lbg' in bounds:
                    constraint['bounds'][cnsrt_elem]['lbg'] = bounds['lbg']
                if 'ubg' in bounds:
                    constraint['bounds'][cnsrt_elem]['ubg'] = bounds['ubg']

        elif isinstance(bounds, list):
            for bound in bounds:
                if 'nodes' not in bound:
                    raise Exception('Missing required element "nodes".')
                else:
                    # if there are more than one chunks of constraint nodes
                    if n_chunks > 1:
                        set_check = set()
                        for el in nodes:
                            if isinstance(el, list):
                                set_check.update(list(range(el[0],el[1])))
                            elif isinstance(el, int):
                                set_check.add(el)

                        if isinstance(bound['nodes'], list):
                            if not set(list(range(bound['nodes'][0], bound['nodes'][1]))).issubset(set_check):
                                raise Exception('List of bound nodes outside constraints nodes.')
                        elif isinstance(bound['nodes'], int):
                            if not {bound['nodes']}.issubset(set_check):
                                raise Exception('Bounds node outside constraints nodes.')
                    else:
                        if isinstance(bound['nodes'], list):
                            if not set(list(range(bound['nodes'][0], bound['nodes'][1]))).issubset([nodes]):
                                raise Exception('List of bound nodes outside constraints nodes.')
                            elif isinstance(bound['nodes'], int):
                                if not {bound['nodes']}.issubset([nodes]):
                                    raise Exception('Bounds node outside constraints nodes.')

                    if isinstance(bound['nodes'], list):
                        for node in range(bound['nodes'][0], bound['nodes'][1]):
                            constraint['bounds'][node] = dict(lbg=bound['lbg'], ubg=bound['ubg'])
                    elif isinstance(bound['nodes'], int):
                            constraint['bounds'][bound['nodes']] = dict(lbg=bound['lbg'], ubg=bound['ubg'])

        self.g_dict[name] = constraint

    def addConstraint(self, name):

        # print('name:', name)
        # print('f:', self.g_dict[name]['constraint'])
        # print('var_opt:', self.var_opt)
        # print('vars:', [self.var_opt[x] for x in self.g_dict[name]['var']])
        # print('g_dict:', self.g_dict[name])

        f = self.g_dict[name]['constraint']
        g = f(*[self.var_opt[x] for x in self.g_dict[name]['var']])
        # print('g: {} {}'.format(name, g.shape))
        # print('value:', g)
        # print('bounds: {}'.format(self.g_dict[name]['bounds']))
        self.g.append(g)

    def setConstraintBounds(self, lbg, ubg, nodes):

        self.lbg[nodes[0]:nodes[1]] = [lbg] * (nodes[1] - nodes[0])
        self.ubg[nodes[0]:nodes[1]] = [ubg] * (nodes[1] - nodes[0])

    def setConstraintBoundsFromName(self, name, nodes, ubg, lbg):

        if isinstance(nodes, list):
            for node in range(nodes[0], nodes[1]):
                self.g_dict[name]['bounds'][node]['lbg'] = lbg
                self.g_dict[name]['bounds'][node]['ubg'] = ubg
        elif isinstance(nodes, int):
            self.g_dict[name]['bounds'][nodes]['lbg'] = lbg
            self.g_dict[name]['bounds'][nodes]['ubg'] = ubg

        # todo can speed up, this is just rewriting everything everytime
        self.lbg = list()
        self.ubg = list()
        self.addConstraintBounds()

    def addConstraintBounds(self):

        for node in range(self.N):
            for constraint in self.g_dict:
                if node in self.g_dict[constraint]['bounds']:
                    self.lbg.extend(self.g_dict[constraint]['bounds'][node]['lbg'])
                    self.ubg.extend(self.g_dict[constraint]['bounds'][node]['ubg'])

                # print('{0}_lbg --> nodes: {1} bounds: {2}'.format(constraint, bound, self.g_dict[constraint]['bounds'][bound]['lbg']))
                # print('{0}_ubg --> nodes: {1} bounds: {2}'.format(constraint, bound, self.g_dict[constraint]['bounds'][bound]['ubg']))


            # # for k in list(self.g_dict[constraint]['bounds']['lbg'].keys()):
            # #     self.lbg.extend(self.g_dict[constraint]['bounds']['lbg'][k])
            #
            # for k in list(self.g_dict[constraint]['bounds']['ubg'].keys()):
            #     self.ubg.extend(self.g_dict[constraint]['bounds']['ubg'][k])
            #

    def getConstraints(self):
        return cs.vertcat(*self.g)

    def update(self, var):
        self.var_opt = var


class StepSolver:

    def __init__(self):

        self.n_duration = 0.02

        self.initial_ds_t = 0.2  # 0.2
        self.ss_1_t = 0.5  # 0.5
        self.ds_1_t = 0.
        self.ss_2_t = 0.
        self.final_ds_t = 0.4  # 0.4

        self.T_total = self.initial_ds_t + self.ss_1_t + self.ds_1_t + self.ss_2_t + self.final_ds_t
        self.N = int(self.T_total / self.n_duration)  # number of control intervals

        self.height_com = 1

        print('duration of initial_ds_t: {}'.format(self.initial_ds_t))
        print('duration of ss_1_t: {}'.format(self.ss_1_t))
        print('duration of ds_1_t: {}'.format(self.ds_1_t))
        print('duration of ss_2_t: {}'.format(self.ss_2_t))
        print('duration of final_ds_t: {}'.format(self.final_ds_t))
        print('T: {}'.format(self.T_total))
        print('N: {}'.format(self.N))
        print('duration of a single node: {}'.format(self.n_duration))

        self.initial_ds_n = int(self.initial_ds_t / self.n_duration)
        self.ss_1_n = int(self.ss_1_t / self.n_duration)
        self.ds_1_n = int(self.ds_1_t / self.n_duration)
        self.ss_2_n = int(self.ss_2_t / self.n_duration)
        self.final_ds_n = int(self.final_ds_t / self.n_duration)

        # print('duration (in nodes) of initial ds: {}'.format(initial_ds_n))
        # print('duration (in nodes) of first ss: {}'.format(ss_1_n))
        # print('duration (in nodes) of middle ds: {}'.format(ds_1_n))
        # print('duration (in nodes) of second ss: {}'.format(ss_2_n))
        # print('duration (in nodes) of final ds: {}'.format(final_ds_n))

        self.ds_1 = self.initial_ds_n
        self.ss_1 = self.ds_1 + self.ss_1_n
        self.ds_2 = self.ss_1 + self.ds_1_n
        self.ss_2 = self.ds_2 + self.ss_2_n
        ds_3 = self.ss_2 + self.final_ds_n

        self.sym_c = cs.SX

        # state variables
        self.p = self.sym_c.sym('p', 2)  # com position
        self.v = self.sym_c.sym('v', 2)  # com velocity
        self.a = self.sym_c.sym('a', 2)  # com acceleration
        self.x = cs.vertcat(self.p, self.v, self.a)  # , l, r, alpha_l, alpha_r # state
        # control variables
        self.j = self.sym_c.sym('j', 2)  # com jerk
        self.u = self.j  # control
        # model equation
        self.xdot = cs.vertcat(self.v, self.a, self.j)

        # Objective terms
        self.L = cs.sumsqr(self.u)
        # Formulate discrete time dynamics
        # Fixed step Runge-Kutta 4 integrator
        self.M = 1  # RK4 steps per interval
        self.dt = self.T_total / self.N / self.M

        margin = 0.0
        self.width_foot = 0.1 - margin
        self.length_foot = 0.2 - margin

        self.max_stride_x = 0.4
        self.max_stride_y = self.width_foot / 2. + 0.5 #0.3 #
        self.min_stride_y = self.width_foot / 2. + 0.15

        self.grav = 9.81

    def getEdges(self, p):  # lu, ru, rh, lh

        lu = cs.DM([+ self.length_foot / 2., + self.width_foot / 2.])
        ru = cs.DM([+ self.length_foot / 2., - self.width_foot / 2.])
        rh = cs.DM([- self.length_foot / 2., - self.width_foot / 2.])
        lh = cs.DM([- self.length_foot / 2., + self.width_foot / 2.])

        ft_vert = cs.horzcat(p + lu, p + ru, p + rh, p + lh).T

        return ft_vert

    def stepPattern(self, prb_vars, prb_funs, initial_n, final_n, type_leg):

        if type_leg == 'D':
            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_zmp_stab', prb_funs['zmp'] - (
                    casadi_sum(prb_funs['wl_vert'], 0).T + casadi_sum(prb_funs['wr_vert'], 0).T),
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_contacts',
                                              casadi_sum(prb_vars['alpha_l'], 0).T + casadi_sum(prb_vars['alpha_r'], 0).T,
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[1.], ubg=[1.]))
            if initial_n == 0:
                initial_n = 1

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_l_foot', prb_vars['l'] - prb_vars['l-1'], nodes=[initial_n, final_n],
                                              bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))
            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_r_foot', prb_vars['r'] - prb_vars['r-1'], nodes=[initial_n, final_n],
                                              bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

        else:
            if type_leg.lower() == 'r':
                other_leg = 'l'
            else:
                other_leg = 'r'

            stance_w_vert = 'w' + other_leg + '_vert'
            stance_alpha = 'alpha_' + type_leg.lower()
            swing_alpha = 'alpha_' + other_leg
            fixed_foot = other_leg

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_zmp_stab', prb_funs['zmp'] - casadi_sum(prb_funs[stance_w_vert], 0).T,
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_contacts_' + stance_alpha, casadi_sum(prb_vars[stance_alpha], 0).T,
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[0.], ubg=[0.]))

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_contacts_' + swing_alpha, casadi_sum(prb_vars[swing_alpha], 0).T,
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[1.], ubg=[1.]))

            self.prb.ct.setConstraintFunction(type_leg + '_' + str(final_n) + '_r_foot', prb_vars[fixed_foot] - prb_vars[fixed_foot + '-1'],
                                              nodes=[initial_n, final_n], bounds=dict(lbg=[0., 0.], ubg=[0., 0.]))


    def buildProblemStep(self):

        self.prb = Problem(self.N, crash_if_suboptimal=True)

        self.prb.setVariable('x', 6)
        self.prb.setVariable('x-1', 6)

        self.prb.setVariable('l', 2)
        self.prb.setVariable('r', 2)

        self.prb.setVariable('l-1', 2)
        self.prb.setVariable('r-1', 2)

        self.prb.setVariable('alpha_l', 4)
        self.prb.setVariable('alpha_r', 4)

        self.prb.setVariable('u', 2)
        self.prb.setVariable('u-1', 2)
        prb_vars = self.prb.getVariable()

        # zmp variable
        zmp = prb_vars['x'][0:2] - prb_vars['x'][4:6] * (self.height_com / self.grav)
        self.prb.setFunction('zmp', zmp)

        # integrator for multiple shooting
        integrator = RK4(self.M, self.L, self.x, self.u, self.xdot, self.dt)
        x_int = integrator(x0=prb_vars['x-1'], p=prb_vars['u-1'])

        self.prb.setFunction('x_int', x_int)

        wl_vert = prb_vars['alpha_l'] * self.getEdges(prb_vars['l'])
        wr_vert = prb_vars['alpha_r'] * self.getEdges(prb_vars['r'])

        self.prb.setFunction('wl_vert', wl_vert)
        self.prb.setFunction('wr_vert', wr_vert)

        prb_funs = self.prb.getFunction()

        ds_n_1 = self.initial_ds_n + 1
        ss_n_1 = self.ds_1 + self.ss_1_n + 1
        ds_n_2 = self.ss_1 + self.ds_1_n + 1
        ss_n_2 = self.ds_2 + self.ss_2_n + 1
        ds_n_3 = self.N + 1

        # todo remember that the last node is N+1! change something
        # add constraints
        self.prb.ct.setConstraintFunction('multiple_shooting', prb_funs['x_int']['xf'] - prb_vars['x'], nodes=[1, self.N + 1],
                                     bounds=dict(ubg=[0., 0., 0., 0., 0., 0.], lbg=[0., 0., 0., 0., 0., 0.]))
        # self.prb.ct.setConstraintFunction('stride', prb_vars['l'] - prb_vars['r'],
        #                              bounds=dict(lbg=[-self.max_stride_x, -self.max_stride_y], ubg=[self.max_stride_x, self.max_stride_y]))

        # self.stepPattern(prb_vars, prb_funs, 0, ds_n_3, 'D')
        # # step pattern
        self.stepPattern(prb_vars, prb_funs, 0, ds_n_1, 'D')
        self.stepPattern(prb_vars, prb_funs, ds_n_1, ss_n_1, 'L')
        self.stepPattern(prb_vars, prb_funs, ss_n_1, ds_n_2, 'D') # this is zero right now
        self.stepPattern(prb_vars, prb_funs, ds_n_2, ss_n_2, 'R') # this is zero right now
        self.stepPattern(prb_vars, prb_funs, ss_n_2, ds_n_3, 'D')

        # add cost functions
        # self.prb.setCostFunction('minimize_input', 0.001 * cs.sumsqr(prb_vars['u']), nodes=[0, self.N]) # todo trim to lenght of specific node (here 'u')
        self.prb.setCostFunction('minimize_l_motion', cs.sumsqr(prb_vars['l'] - prb_vars['l-1']), nodes=[1, self.N+1])
        self.prb.setCostFunction('minimize_r_motion', cs.sumsqr(prb_vars['r'] - prb_vars['r-1']), nodes=[1, self.N+1])
        # self.prb.setCostFunction('minimize_velocity', cs.sumsqr(prb_vars['x'][2:4]))
        # self.prb.setCostFunction('minimize_zmp', 1000 * cs.sumsqr(prb_funs['zmp']))
        # self.prb.setCostFunction('minimize_alpha', cs.sumsqr(prb_vars['alpha_l']) + cs.sumsqr(prb_vars['alpha_r']))
        # self.prb.setCostFunction('minimize_stride_y', 1000. * cs.sumsqr((prb_vars['l'][1] - prb_vars['r'][1]) - self.min_stride_y))

        self.w, self.g = self.prb.buildProblem()

    def solveProblemStep(self, initial_com, initial_l_foot, initial_r_foot):

        initial_lbw_com = [initial_com[0, 0], initial_com[0, 1],  # com pos
                           initial_com[1, 0], initial_com[1, 1],  # com vel
                           initial_com[2, 0], initial_com[2, 1]]

        initial_ubw_com = [initial_com[0, 0], initial_com[0, 1],
                           initial_com[1, 0], initial_com[1, 1],
                           initial_com[2, 0], initial_com[2, 1]]

        final_lbw_com = [-cs.inf, -cs.inf,
                         0.0, 0.0,
                         0.0, 0.0]

        final_ubw_com = [cs.inf, cs.inf,
                         0.0, 0.0,
                         0.0, 0.0]

        # todo check if lenght of ubw and lbw are of the right size

        self.prb.setStateBoundsFromName('x', nodes=0, lbw=initial_lbw_com, ubw=initial_ubw_com)

        self.prb.setStateBoundsFromName('l', nodes=0, lbw=[initial_l_foot[0], initial_l_foot[1]],
                                         ubw=[initial_l_foot[0], initial_l_foot[1]])
        self.prb.setStateBoundsFromName('r', nodes=0, lbw=[initial_r_foot[0], initial_r_foot[1]],
                                        ubw=[initial_r_foot[0], initial_r_foot[1]])

        self.prb.setStateBoundsFromName('x', nodes= self.N, lbw=final_lbw_com, ubw=final_ubw_com)

        self.prb.setStateBoundsFromName('alpha_l', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])
        self.prb.setStateBoundsFromName('alpha_r', lbw=[0., 0., 0., 0.], ubw=[1., 1., 1., 1.])

        self.prb.setStateBoundsFromName('u', nodes=[0, self.N], lbw=[-1000., -1000.], ubw=[1000., 1000.])

        self.prb.setInitialGuess('l', nodes=[0, self.N+1], vals=[initial_l_foot[0], initial_l_foot[1]])
        # self.prb.setInitialGuess('r', nodes=[0, self.N+1], vals=[initial_r_foot[0], initial_r_foot[1]])

        w_opt = self.prb.solveProblem()

        return w_opt

if __name__ == '__main__':

    solver = StepSolver()
    solver.buildProblemStep()

    a = np.array([[0, 0], [0, 0], [0, 0]])
    b = np.array([-0.2, 0, 0])
    c = np.array([0.2, 0, 0])
    opt_values = solver.solveProblemStep(a, b, c)

    print(opt_values)
    # N = 5
    # prb = Problem(N, crash_if_suboptimal=False)
    # h = 1
    # grav = 9.8

    # a = cs.SX.sym('a', 2)  # com acceleration
    # define state variables
    # x = cs.vertcat(p, v, a)

    # define control variables
    # j = cs.SX.sym('j', 2)  # com jerk
    # u = j  # control
    # model equation
    # xdot = cs.vertcat(v, a, j)

    # integrator = RK4(1, 1, x, u, xdot, 0.01)

    # prb.setVariable('x', 6)
    # prb.setVariable('u', 2)
    # todo check if dimension of past x is the same as x or change api
    # prb.setVariable('x', 6, -1)  # get x-2, which is x two nodes before
    # prb.setVariable('x', 6, -2) # get x-2, which is x two nodes before

    # prb.setVariable('k', 9)
    # var_opt = prb.getVariable()

    # zmp_old = var_opt['x-1'][0:2] - var_opt['x-1'][4:6]  # * (h / grav)
    # zmp = var_opt['x'][0:2] - var_opt['x'][4:6]  # * (h / grav)
    #
    # prb.setFunction('zmp', zmp)
    # prb.setFunction('zmp_old', zmp_old)
    #
    # Fk = integrator(x0=var_opt['x-1'][0:6], p=var_opt['u-1'])

    # fun_opt = prb.getFunction()

    # print(fun_opt)
    # if k > 0:
    # forward integration
    ## Multiple Shooting (the result of the integrator [XInt[k-1]] must be the equal to the value of the next node)
    # prb.ct.addConstraint('multiple_shooting', Fk['xf'] - var_opt['x'][0:6])
    # define template constraint function
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0,2], [3,4]], bounds=(dict(lbg=[-1,-1], ubg=[1,1])))
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(nodes=[0,2]))) # should be wrong
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]], bounds=(dict(ubg=[1, 1])))
    # prb.ct.setConstraintFunction('generic_constraint1', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]])
    # prb.ct.setConstraintFunction('generic_constraint2', var_opt['u'][0:2] - var_opt['x'][4:6], nodes=[[0, 2], [3, 4]])
    # prb.ct.setConstraintFunction('generic_constraint3', var_opt['x-1'][0:2] - var_opt['x'][4:6], nodes=[2, 5])

    # prb.ct.setConstraintFunction('generic_constraint',
    #                              var_opt['x'][0:2] - var_opt['x'][4:6],
    #                              nodes=[[0, 2], [3, 4]],
    #                              bounds=[dict(nodes=[0, 2], lbg=[-1, -1], ubg=[1, 1]), dict(nodes=1, lbg=[-2, -2], ubg=[2, 2])])

    # TODO if nodes are just a number, think
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], nodes=[0, 2])
    # prb.ct.setConstraintFunction('yet_another_constraint', var_opt['u'][0] - var_opt['u'][1], bounds=(dict(ubg=[1], lbg=[-1])))
    # prb.ct.setConstraintFunction('generic_constraint', var_opt['x'][0:2] - var_opt['x'][4:6], bounds=dict(nodes=[2,4], ubg=[1, 1], lbg=[-1,-1]))
    # prb.ct.setConstraintFunction('another_constraint', var_opt['u'] - var_opt['x'][4:6]) #nodes = [[0,2], [4,5]]
    # prb.ct.setConstraintFunction('zmp_constraint', fun_opt['zmp_old'] - var_opt['u'], nodes=[2, prb.N])

    # 1000. * sumsqr((Lk[1] - Rk[1]) - self.min_stride_y)
    # prb.setCostFunction('one_cost_function', fun_opt['zmp'][0] - var_opt['x'][2])
    #
    # problem = prb.buildProblem()
    #
    # todo add check for lenght of value inserted
    # todo add check for lenght of nodes inserted
    # prb.setInitialGuess('u', [0,N], [1, 1])

    # prb.setStateBoundsFromName(name='x', nodes=[0, 3], lbw=[0, 0, 0, 0, 0, 0], ubw=[0, 0, 0, 0, 0, 0])

    # print(w)
    # prb.ct.setConstraintBounds(-5, 5, [3, 4])

    # print(prb.ct.lbg)
    # print(prb.ct.ubg)
    # print('=======================')
    # prb.ct.setConstraintBoundsFromName('generic_constraint3', nodes=2, lbg=[-7.5, -7.5], ubg=[7.5, 7.5])
    # print('=======================')
    # print(prb.ct.g)
    # print(prb.ct.lbg)
    # print(prb.ct.ubg)

    # x and u should be always present

    # w_opt = prb.solveProblem()