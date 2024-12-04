.. horizon documentation master file, created by
   sphinx-quickstart on Sat Aug  7 11:04:36 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

.. include:: <isonum.txt>

Welcome to Horizon's documentation!
===================================
Horizon is a framework for trajectory optimization and optimal control tailored to robotic systems.
It relies on direct methods to reduce an optimization problem into a NLP that can be solved using many state-of-the-art solvers.

Horizon is based on `CasADi <https://web.casadi.org/>`_, a tool for nonlinear optimization and algorithmic differentiation.
It uses `Pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ to smoothly integrate the robot model into the optimization problem.
The structure of Horizon is described :ref:`here <horizon_scheme>`.

Features
========
- complete **pipeline** from model aquisition to robot deployment 
- **intuitive** API allowing a quick setup of the optimization problem
- ease of configuration and highly *customizable*: integrators, transcription methods, solvers..
- support **state-of-the-art** non linear solvers + two custom solvers: ILQR and GNSQP

.. image:: spot_leap.png
   :scale: 200 %
   :alt: the quadruped robot Spot from BostonDynamics perfoming a leap
   :align: center
   
Install
=======
Two distributions are supported:

- *pip* package: ``pip install casadi_horizon``
- *conda* package: ``conda install horizon -c francesco_ruscelli``

Getting started
=======
Some examples demonstrating trajectory optimization for different robots are available.
Besides installing Horizon on your machine and running the examples, you can try out the framework in independent evinronments:

- on your browser, through **JupyterLab**: `horizon-web <https://mybinder.org/v2/gh/FrancescoRuscelli/horizon-live/main?urlpath=lab/tree/index.ipynb>`_
- on your machine, through **Docker**: :ref:`horizon-docker <horizon_docker>`



Videos
=======
A collection of clips demonstrating the capabilities of Horizon is gathered in the video below, where are gathered trajectory optimization demos with several robots: 
*Centauro*, *Spot* |reg|, *Kangaroo*, *TALOS*, a 7-DoF industrial manipulator and a prototype 2-DoF leg.

.. raw:: html

   <p align="center">
   <iframe width="560" height="315" src="https://www.youtube.com/embed/wHJh3bvV-ns" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
   </p>

The full playlist of videos is found `here <https://www.youtube.com/playlist?list=PL7c1ZKncPan72ef2Sof8Ky_TrlSK9qYYP>`_.
 
.. toctree::
   :maxdepth: 4
   :caption: Contents:
   
   horizon
   scheme
   docker

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`