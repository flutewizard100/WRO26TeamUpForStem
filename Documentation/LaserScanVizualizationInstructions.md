1. Launch the WRORobot package in the ssh terminal

2. Make sure you have the same ROS_DOMAIN_ID as the robot (should be 0) and are connected to the same wifi (may have to set up the ROS_DOMAIN_ID and such in the ~/.bashrc)

3. check by running ros2 node list on a non-ssh terminal. You should still be able to see the same ros2 node list

4. In the non-ssh terminal run rviz2 command

5. Then change the fixed frame to be base_link

6. Add laser scan display to the rviz2

7. If you want to make the dots more bigger or stable, there are some settings underneath the laser scan display (increasing Decay time to 1 should make it more stable) and you can increase the size setting
