# PLEXI
Plan Execution Interface

Luca Iocchi, Sapienza University of Rome, Italy (2021)

----

Plan Execution Interface (PLEXI) is a layer for increasing interoperability between a plan exeuction monitor (PEM) and the implementation of actions and fluents (or conditions) in a complex system (e.g., a robotic application). PLEXI provides an interaction protocol between PEM and actions/fluents implementations, allowing for decoupling all these components. 

Implementations of PLEXI are provided for both ROS (Python and C++) and gRPC (Python and C++) -- *ongoing work...*

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





