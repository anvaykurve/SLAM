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
# Checkpoint 3 

Task 3.1
```
cd ~/SLAM_Module/src/Checkpoint3/src/
g++ data_association.cpp -o test_association && ./test_association
```
Task 3.2
```
cd ~/SLAM_Module/src/Checkpoint3/src/
g++ -I/usr/include/eigen3 ml_association.cpp -o test_ml && ./test_ml
```
Task 3.3
```
cd ~/SLAM_Module/src/Checkpoint3/src/
g++ kdtree_association.cpp -o test_kdtree && ./test_kdtree
```
# Checkpoint 4
Terminal 1
```
cd ~/SLAM_Module
colcon build --packages-select Checkpoint4
source install/setup.bash
ros2 bag play src/Checkpoint2/trainee_bag_cp2_0.db3 --clock --loop
```

