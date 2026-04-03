# Checkpoint 2 
Terminal 1
```
cd ~/SLAM_Module
colcon build --packages-select Checkpoint2
source install/setup.bash
```
Teminal 2
```
ros2 bag play src/Checkpoint2/trainee_bag_cp2_0.db3 --clock --loop
```
Terminal 3
```
rviz2 --ros-args -p use_sim_time:=true
```
