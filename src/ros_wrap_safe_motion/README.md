# This package is for generating collision free trajectory in human-robot collaborative task 

The package is self-containing but depending few open-source package such as [**Eigen**](https://gitlab.com/libeigen/eigen.git), [**qpOASES**](https://github.com/kuka-isir/qpOASES). [**rviz_visual_tools**](https://github.com/PickNikRobotics/rviz_visual_tools). And it simulates result of the paper [**Online optimal motion generation with guaranteedsafety in shared workspace**](https://hal.archives-ouvertes.fr/hal-02496057/document)


## config 

parameter as controller PID gains, controller type and other information such as topic's name etc 

## launch

human_work.launch will launch node as Gazebo, rviz, controller, robot's urdf, MPController and separating plane node.

## schema of control

First we start robot motion with a disired task and robot's intrinsec constraint such as position, velocity etc with node name **MPController**, this node publish data such as robot's vertices augmented via topic "MPController/Panda/robotVertices". This node also subcribe to topic "/plane_data" to read separating plane data.

Secondly, we run **separating Plane node**, this node publisher data to "/plane_data" and subcriber to robotVertices and Obstacle verticee. 

The end, we use camera to detect obstacle's position and send it to separating plane node. (Not implemented yet, using some simulate obstacle's location)
