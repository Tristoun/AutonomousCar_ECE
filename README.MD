## REQUIREMENTS

1. Ros2 humble on ubuntu 22.04
2. Microros for ros2 humble WITH microros agent
3. Check your wifi config if you're using a vm

## Launch the project (ros2 computer side)

You need to change wifi config before launching the project

```bash
cd ros2_ws
source install/setup.bash
#The next line will launch the robotdescription package with foxglove bridge and micro ros agent you must source microros_ws before launch the package
ros2 launch my_robot_description robot_description.launch.py 

#If you need to source microros_ws
cd ~/microros_ws
source install/setup.bash
```

