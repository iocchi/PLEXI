# PLEXI
Plan Execution Interface

Luca Iocchi, Sapienza University of Rome, Italy (2021)

----

Plan Execution Interface (PLEXI) is a layer for increasing interoperability between a plan exeuction monitor (PEM) and the implementation of actions and fluents (or conditions) in a complex system (e.g., a robotic application). PLEXI provides an interaction protocol between PEM and actions/fluents implementations, allowing for decoupling all these components. 

Implementations of PLEXI are provided for both ROS (Python and C++) and gRPC (Python and C++) -- (*ongoing work...*)

With PLEXI, action/fluent implementations will not depend from the specific PEM. There is no need to import libraries or include code from the PEM,
there is no need to know with PEM will orchestrate the actions/fluents.

Interaction between PEM and actions/fluents will exploit the FLEXI protocol using ROS and gRPC communication channels.
ROS/gRPC Python/C++ implementations can be mixed in the same application. 
All implemented actions/fluents, each one using its own language (Python/C++) and communication channel (ROS/gRPC), will be managed by the same PEM.


## PLEXI protocol specification

PLEXI protocol is based on string messages with the following semantics:

----

Message specifications

Action interface

From PEM to PLEXI

* start
* interrupt
* resume

Form PLEXI to PEM

* end

Fluent interface

From PEM to PLEXI

* getValue

From PLEXI to PEM

* value (with timestamp of last obesrvation)


## PLEXI integration in actions/fluents implementation

To add PLEXI interface to an action/fluent implementation:

* copy the actionproxy/fluentproxy base classes from this repository in your application
* define a specific class (see the templates as examples)
* run the specific class

The specific class will act as a wrapper/proxy to the code implementing the action/fluent enabling it to be controlled by a FLEXI-enabled PEM.
