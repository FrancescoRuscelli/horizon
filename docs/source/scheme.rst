.. _horizon_scheme:

Horizon scheme
=====================

A pseudo-UML representation of the structure of Horizon.

The core component is the *Problem* component, acting as a manager for optimization variables, parameter, costs and constraints. 
Variables can be combined thanks to CasADi's symbolic engine to produce expressions for costs, and constraints. 
In the robotics domain, dynamics, costs, and constraints can contain expressions related to kinematics and dynamics, as allowed by the *ModelParser* component. 
A *TranscriptionMethod* component augments the Problem with variables and constraints implementing a transcription strategy. 
Finally, a *Solver* component translates the abstract representation of Problem into an actual NLP, which is solved to produce a numerical result.

.. image:: horizon.jpg
   :scale: 200 %
   :alt: a pseudo-UML representation of the structure of Horizon.
   :align: center