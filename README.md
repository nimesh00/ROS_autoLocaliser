!! This is the complete catkin workspace
HOW TO USE:

* Make sure you have the turtlebot3* package insatlled.
* Set the turtlebot3 model environment variable (export TURTLEBOT3_MODEL=burger).
* Build the workspace (catkin build).
* Update the initial pose values in the pose.txt file in the localiser package directory (Only needed for the start).
* Launch the AMR world (roslaunch localiser amr_world.launch).
* Launch the AMR navigation node (roslaunch localiser amr_navigation.launch).
* Move the AMR (teleop or any other method).
* Close and relaunch the amr_navigation.launch node.
