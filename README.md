Computed Torque with iCub
========================

# Prerequisites

To fully complete this assignment you should be familiar to the following components:

- (Theory) Computed torque control
- The [`remotecontrolboardremapper`](http://www.yarp.it/classyarp_1_1dev_1_1RemoteControlBoardRemapper.html) YARP device 
- YARP ports 
- [`iDynTree::Model`](http://robotology.gitlab.io/docs/idyntree/master/classiDynTree_1_1Model.html), 
[`iDynTree::ModelLoader`](http://robotology.gitlab.io/docs/idyntree/master/classiDynTree_1_1ModelLoader.html) 
and [`iDynTree::KinDynComputations`](http://robotology.gitlab.io/docs/idyntree/master/classiDynTree_1_1KinDynComputations.html) iDynTree classes

# Assignment
You should develop a computed torque controller for the iCub humanoid robot.

Joint position references for the controller come through a streaming port (call it `/{yourmodulename}/qDes:i`).

The robot will be fixed in his root link (i.e. the pelvis), so you are not required to control the full free floating dynamics. Note however that the functions refer to a free floating dynamics, so you have to pay particular care to the variables size.

# Note

We provide a skeleton of the YARP controller with the methods partially implemented. 
In particular we implemented for you the initialization of the `remotecontrolboardremapper` YARP device and the `iDynTree::KinDynComputations` object.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
