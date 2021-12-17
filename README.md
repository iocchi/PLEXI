# PLEXI
Plan Execution Interface

Luca Iocchi, Sapienza University of Rome, Italy (2021)

----

A Plan Execution Monitor (PEM) aims at orchestrating the execution of actions and monitor the values of fluents (or predicates), following a plan generated to achieve a given goal.
The implementation of the interface between a PEM and action/fluent implementations may be non-trivial and it is usually achieved through specific coupling PEM and action/fluent implementations.
Existing examples are ROSPlan Action and Sensing interfaces (https://kcl-planning.github.io/ROSPlan/documentation/) and PetriNetPlans ActionServers (https://github.com/iocchi/PetriNetPlans/tree/master/PNPros/ROS_bridge/pnp_ros).

Plan Execution Interface (PLEXI) is a layer for increasing interoperability between a PEM and the implementation of actions and fluents in a complex system (e.g., a robotic application). PLEXI provides an interaction protocol between PEM and action/fluent implementations, allowing for decoupling all these components. 

With PLEXI, action/fluent implementations will not depend on the specific PEM. There is no need to import libraries or include code from the PEM, there is no need to know which PEM will orchestrate the implemented actions/fluents.

Implementations of PLEXI are provided for both ROS (Python and C++) and gRPC (Python and C++) -- (*ongoing work...*)

Interaction between PEM and actions/fluents will use the FLEXI protocol over ROS and gRPC communication channels.
ROS/gRPC Python/C++ implementations can be mixed in the same application. 
All implemented actions/fluents, each one using its own language (Python/C++) and communication layer (ROS/gRPC), will be managed by the same PEM.


## PLEXI protocol specification

PLEXI protocol is based on string messages with the following semantics:

----

### Action messages

From PEM to PLEXI

* start
* interrupt
* resume

From PLEXI to PEM

* end (with success/failure state)

### Fluent messages

From PEM to PLEXI

* getValue

From PLEXI to PEM

* value (with timestamp of last obesrvation)


## PLEXI wrappers for action/fluent implementation

To add PLEXI interface to an action/fluent implementation:

* copy the actionproxy/fluentproxy base classes from this repository to the action/fluent implementation
* define a specific proxy for each action/fluent (see the templates as examples)
* start all the acion/fluent proxies

The proxies will operate as interfaces between the PEM and the action/fluent specific code.
A process will be running for each proxy to communicate with the PEM and implement actual plan execution.


A full example of use of this interface can be found in (https://github.com/iocchi/DIAG_demo).
