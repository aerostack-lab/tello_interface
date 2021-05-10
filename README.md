# Tello Interface

The ROS node "tello interface" implements a ROS bridge for the DJI Tello drone. 

# Subscribed topics

- **actuator_command/flight_action** ([aerostack_msgs/FlightActionCommand](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/7c07e4317e20a1142226d513336a06a2ff585629/msg/FlightActionCommand.msg))      
Flight command specifying a qualitative action of the robot (take off, hover or land). This command can be interpreted by some aerial platforms (e.g. Parrot drones) but it is ignored by others.

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **actuator_command/roll_pitch_yaw_rate_thrust** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the multirotor specifying roll (rad), pitch (rad), yaw rate (rad/s) and thrust (N: Newtons).

# Published topics

- **sensor_measurement/linear_speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))           
Linear and angular speeds along roll, pitch and yaw axes. Units are m/s and rad/s respectively.

- **sensor_measurement/imu** ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))           
IMU measurement.

- **sensor_measurement/battery_state** ([sensor_msgs/BatteryState](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html))           
Battery measurement.

- **sensor_measurement/temperature** ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html))           
Temperature measurement.

- **sensor_measurement/altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))           
Altitude measurement in meters.

- **sensor_measurement/sea_level_altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))           
Sea level measurement in meters.

- **self_localization/flight_state** ([aerostack_msgs/FlightState](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/7c07e4317e20a1142226d513336a06a2ff585629/msg/FlightState.msg))           
Flight state (landed, hovering, ...).

- **sensor_measurement/camera** ([sensor_msgs/Camera](http://docs.ros.org/api/sensor_msgs/html/msg/Camera.html))           
Camera video.