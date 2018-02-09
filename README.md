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

Joint position references for the controller come through a streaming port (call it `/computed-torque/qDes:i`, pay attention as the test will assume this name!). **Note: the controller will expect this references in radians, differently from most YARP-software that send angular quantities expressed in degrees.**

The robot will be fixed in his root link (i.e. the pelvis, or `root_link` in the URDF), so you are not required to control the full free floating dynamics. Note however that the functions refer to a free floating dynamics, so you have to pay particular care to the variables size.

## Requirements to meet
The test will send a new reference to your controller (on the `/computed-torque/qDes:i` port)
and it will pass if:
1. your controller is able to track that reference with a maximum error (at steady state) of **5 degrees**.  

### Score map

| Requirements | Points |
|:------------:|:------:|
| 1             | 5 |
| **Maximum score** | 5 |


# How to proceed

We provide a skeleton of the YARP controller with the methods partially implemented. 
In particular we implemented for you the initialization of the `remotecontrolboardremapper` YARP device and the `iDynTree::KinDynComputations` object. 
To find the parts in which you need to insert the code, look out for the `// FILL IN THE CODE` comments. 

Once done, you can test your code in two ways:
* **Manually:** running the yarpmanager scripts provided from within app/scripts. This will help you interact with your code.
* **Automatically:** running the script test.sh in the smoke-test directory. 
    
# Tips and tricks  
*  If you need to get the output of your controller, you can use `yarplogger` also when running the smoke-test. Just launch the yarpserver, the yarplogger and start the yarplogger before launching the `./test.sh` script. The output of the nodes launched by the fixture of the test will appear in the `yarplogger`, while the output of the test itself will be printer in the terminal on which you launched the `./test.sh` script.
* There are a few known issues affecting the software used in this assignement. This known issue are not preventing you from completing the assignment, but they can be confusing. Please check the relevant issues to get more informations : 
  * https://github.com/vvv-school/assignment_computed-torque/issues/3
  * https://github.com/vvv-school/assignment_computed-torque/issues/4
  

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
