# rosduct

*ROSduct*, the duct of ROS messages. ROSduct acts as a proxy to expose ROS topics, services and parameters from a remote `roscore` into a local `roscore` via the [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md).

Say you have a ROS enabled robot in your network and you want to communicate with it but you have a network configuration that does not allow direct communication (for example, from inside a Docker container). With ROSduct you can configure a set of topics, services and parameters (action servers too, they are implemented as topics) to expose in the local roscore to transparently send and receive ROS traffic to the robot ones.

TODO: Image explaining it.

# Usage
Fill up YAML file with your topic publishers, subscribers, service servers to access, service servers to expose and parameters. Also the IP and port of the ROSbridge websocket server.

```yaml
# ROSbridge websocket server info
rosbridge_ip: 192.168.1.31
rosbridge_port: 9090
# Topics being published in the robot to expose locally
remote_topics: [
                    ['/joint_states', 'sensor_msgs/JointState'], 
                    ['/tf', 'tf2_msgs/TFMessage'],
                    ['/scan', 'sensor_msgs/LaserScan']
                    ]
# Topics being published in the local roscore to expose remotely
local_topics: [
                    ['/test1', 'std_msgs/String'],
                    ['/closest_point', 'sensor_msgs/LaserScan']
                    ]
# Services running in the robot to expose locally
remote_services: [
                    ['/rosout/get_loggers', 'roscpp/GetLoggers']
                    ]
# Services running locally to expose to the robot
local_services: [
                    ['/add_two_ints', 'beginner_tutorials/AddTwoInts']
                    ]
# Parameters to be sync, they will be polled to stay in sync
parameters: ['/robot_description']
parameter_polling_hz: 1
```