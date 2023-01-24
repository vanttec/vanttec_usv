## Velocity Obstacles

Velocity obstacles is a method used for computing avoidance velocities for static and dynamic obstacles.
The implementation proposed here considers only static and round-shaped obstacles; currentlty for a single obstacle.

To try the method just run in your terminal:

```
roscore
rviz
roslaunch usv_control simulation_avoidance.launch
rosrun usv_control lidar_obstacle_simulator.py
rosrun usv_control velocity_obstacle
rosrun usv_control los_vo.py
```

To visualize the obstacle in Rviz, click the button "Add" and select the "by topic" tab. There, under `/usv_perception/lidar_detector/markers`
select `Marker Array` and click "ok".
