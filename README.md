# rosduct

*ROSduct*, the duct of ROS messages. ROSduct acts as a proxy to expose ROS topics, services and parameters from a remote `roscore` into a local `roscore` via the [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/develop/ROSBRIDGE_PROTOCOL.md).

Say you have a ROS enabled robot in your network and you want to communicate with it but you have a network configuration that does not allow direct communication (for example, from inside a Docker container). With ROSduct you can configure a set of topics, services and parameters (action servers too, as they are implemented internally as topics) to expose in the local roscore to transparently send and receive ROS traffic to the robot ones.

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

**Note**: Don't add to remote or local topics the topic `/rosout`.

# Example usage with Docker
This tool came to be mainly to solve a problem with Docker containers. If you are running the Docker container that wants bidirectional communication with a ROS robot, and you are using Linux you can just add `--net host` to your `docker run` command (just after run). But if you are using a Mac [this won't work](https://github.com/docker/for-mac/issues/68). To work around it you can use this package.

Just get in your Docker image rosduct:

```bash
mkdir -p ~/rosduct_ws/src
cd ~/rosduct_ws/src
git clone https://github.com/uts-magic-lab/rosduct
cd ..
catkin_make
. devel/setup.bash
```

And make a launchfile that configures it to expose your needed topics/services. For example, for a tool that interacts with `move_base` you could have a launchfile like:

```xml
<launch>
  <node pkg="rosduct" name="rosduct" type="rosduct_main.py" output="screen">
    <rosparam>
    # ROSbridge websocket server info
    rosbridge_ip: YOUR.ROBOT.IP.HERE
    rosbridge_port: 9090
    # Topics being published in the robot to expose locally
    remote_topics: [
                        ['/amcl_pose', 'geometry_msgs/PoseWithCovarianceStamped'], 
                        ['/move_base/feedback', 'move_base_msgs/MoveBaseActionFeedback'],
                        ['/move_base/status', 'actionlib_msgs/GoalStatusArray'],
                        ['/move_base/result', 'move_base_msgs/MoveBaseActionResult'],
                        ]
    # Topics being published in the local roscore to expose remotely
    local_topics: [
                        ['/move_base/goal', 'move_base_msgs/MoveBaseActionGoal'],
                        ['/mover_base/cancel', 'actionlib_msgs/GoalID']
                        ]
    # Services running in the robot to expose locally
    remote_services: [
                        ['/move_base/clear_costmaps', 'std_srvs/Empty']
                        ]
    # Services running locally to expose to the robot
    local_services: []
    # Parameters to be sync, they will be polled to stay in sync
    #parameters: []
    #parameter_polling_hz: 1

    </rosparam>
  </node>
</launch>
```

So you run your Docker image exposing port 9090 (for rosbridge communication) `docker run -p 9090:9090 -it your_docker_image` and you run the previous launchfile before running your ROS node.

To build the config you can do `rosnode info YOUR_NODE` and check the Publications (`local_topics`) and Subscriptions (`remote_topics`) and Services (`local_services`). For filling up remote_services you need to know what services your node calls.