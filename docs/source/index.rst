.. horizon documentation master file, created by
   sphinx-quickstart on Sat Aug  7 11:04:36 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to horizon's documentation!
===================================
A framework for trajectory optimization and optimal control tailored to robotic systems.
It relies on direct methods to reduce an optimization problem into a NLP that can be solved using many state-of-the-art solvers.

Horizon is based on `CasADi <https://web.casadi.org/>`_, a tool for nonlinear optimization and algorithmic differentiation.
It uses `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ to smoothly integrate the robot model into the optimization problem.

Features
========
- complete **pipeline** from model aquisition to robot deployment 
- **intuitive** API allowing a quick setup of the optimization problem
- ease of configuration and highly *customizable*: integrators, transcription methods, solvers..
- support **state-of-the-art** non linear solvers

Install
=======
A *pip* package is available: ``pip install casadi_horizon``
..or, alternatively, a *conda* package is supported: ``conda install horizon -c francesco_ruscelli``

Getting started
=======
Some examples demonstrating trajectory optimization for different robots are available.
Besides installing Horizon on your machine and running the examples, you can try out the framework in independent evinronments:

- on your browser, through JupyterLab: `Horizon-web <https://mybinder.org/v2/gh/FrancescoRuscelli/horizon-live/main?urlpath=lab/tree/index.ipynb>`_
- on your machine, through Docker: `Horizon-docker` (tbd)


.. toctree::
   :maxdepth: 2
   :caption: Contents:
   
   modules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
