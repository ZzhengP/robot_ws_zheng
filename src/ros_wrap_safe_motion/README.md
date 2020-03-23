# This package is for generating collision free trajectory in human-robot collaborative task 

The package is self-containing but depending few open-source package such as [**Eigen**](https://gitlab.com/libeigen/eigen.git), [**qpOASES**](https://github.com/kuka-isir/qpOASES). [**rviz_visual_tools**](https://github.com/PickNikRobotics/rviz_visual_tools).


## config 

parameter as controller PID gains, controller type and other information such as topic's name etc 

## launch

human_work.launch will launch node as Gazebo, rviz, controller, robot's urdf, MPController and separating plane node.