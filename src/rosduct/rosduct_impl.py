#!/usr/bin/env python

import rospy
from conversions import from_dict_to_JSON
from conversions import from_JSON_to_dict
from conversions import from_dict_to_ROS
from conversions import from_ROS_to_dict
from conversions import from_JSON_to_ROS
from conversions import from_ROS_to_JSON
from conversions import get_ROS_msg_type
from conversions import get_ROS_class
from conversions import is_ros_message_installed, is_ros_service_installed
from pydoc import locate
import socket

from rosbridge_client import ROSBridgeClient

"""
Server to expose locally and externally
topics, services and parameters from a remote
roscore to a local roscore.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

yaml_config = '''
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
parameter_polling_hz: 1'''


class ROSduct(object):
    def __init__(self):
        # ROSbridge
        self.rosbridge_ip = rospy.get_param('~rosbridge_ip', None)
        if self.rosbridge_ip is None:
            rospy.logerr('No rosbridge_ip given.')
            raise Exception('No rosbridge_ip given.')
        self.rosbridge_port = rospy.get_param('~rosbridge_port', 9090)
        rospy.loginfo("Will connect to ROSBridge websocket: ws://{}:{}".format(
            self.rosbridge_ip, self.rosbridge_port))

        # Topics
        # TODO: check if topic types are installed, if not, give a warning
        self.remote_topics = rospy.get_param('~remote_topics', [])
        rospy.loginfo("Remote topics: " + str(self.remote_topics))
        self.local_topics = rospy.get_param('~local_topics', [])
        rospy.loginfo("Local topics: " + str(self.local_topics))

        # Services
        # TODO: check if service types are installed
        self.remote_services = rospy.get_param('~remote_services', [])
        rospy.loginfo("Remote services: " + str(self.remote_services))
        self.local_services = rospy.get_param('~local_services', [])
        rospy.loginfo("Local services: " + str(self.local_services))

        # Parameters
        self.rate_hz = rospy.get_param('~parameter_polling_hz', 1)
        self.parameters = rospy.get_param('~parameters', [])

        self.check_if_msgs_are_installed()

        self.initialize()

    def general_cb(self, message):
        topic_name = message.get('topic')
        topic_type = 'IDONTKNOW'
        rospy.loginfo("Remote ROSBridge subscriber from topic " +
                      topic_name + ' of type ' +
                      topic_type + ' got data: ' + str(message.get('msg')) +
                      ' which is republished locally.')

    def initialize(self):
        """
        Initialize creating all necessary bridged clients and servers.
        """
        connected = False
        while not rospy.is_shutdown() and not connected:
            try:
                self.client = ROSBridgeClient(
                    self.rosbridge_ip, self.rosbridge_port)
                connected = True
            except socket.error as e:
                rospy.logwarn(
                    'Error when opening websocket, is ROSBridge running?')
                rospy.logwarn(e)
                rospy.sleep(5.0)

        # We keep track of the instanced stuff in this dict
        self._instances = {'topics': [],
                           'servers': []}
        for topic_name, topic_type in self.remote_topics:
            rospub = rospy.Publisher(topic_name,
                                     get_ROS_class(topic_type),
                                     queue_size=1)

            # the keyword arguments are mandatory for the closure
            # to do it's actual job
            def custom_callback_remote_to_local(message,
                                                topic_name=topic_name,
                                                topic_type=topic_type,
                                                rospub=rospub):
                rospy.loginfo("Remote ROSBridge subscriber from topic " +
                              topic_name + ' of type ' +
                              topic_type + ' got data: ' + str(message) +
                              ' which is republished locally.')
                msg = from_dict_to_ROS(message, topic_type)
                rospub.publish(msg)
            bridgesub = self.client.subscriber(
                topic_name, topic_type,
                custom_callback_remote_to_local)
            # self.general_cb)
            self._instances['topics'].append(
                {topic_name:
                 {'rospub': rospub,
                  'bridgesub': bridgesub}
                 })

        for topic_name, topic_type in self.local_topics:
            bridgepub = self.client.publisher(topic_name, topic_type)

            def custom_callback_local_to_remote(message,
                                                topic_name=topic_name,
                                                topic_type=topic_type,
                                                bridgepub=bridgepub):
                rospy.loginfo("Local subscriber from topic " +
                              topic_name + ' of type ' +
                              topic_type + ' got data: ' + str(message) +
                              ' which is republished remotely.')
                dict_msg = from_ROS_to_dict(message)
                bridgepub.publish(dict_msg)

            rossub = rospy.Subscriber(topic_name,
                                      get_ROS_class(topic_type),
                                      custom_callback_local_to_remote)
            self._instances['topics'].append(
                {topic_name:
                 {'rossub': rossub,
                  'bridgepub': bridgepub}
                 })

    def check_if_msgs_are_installed(self):
        """
        Check if the provided message types are installed.
        """
        for _, topic_type in self.remote_topics:
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))

        for _, topic_type in self.local_topics:
            if not is_ros_message_installed(topic_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(topic_type))

        for _, service_type in self.remote_services:
            if not is_ros_service_installed(service_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(service_type))

        for _, service_type in self.local_services:
            if not is_ros_service_installed(service_type):
                rospy.logwarn(
                    "{} could not be found in the system.".format(service_type))

    def sync_params(self):
        """
        Sync parameter server in between
        external and local roscore.
        """
        pass

    def spin(self):
        """
        Run the node, needed to update the parameter server.
        """
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.sync_params()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('rosduct')
    r = ROSduct()
    r.spin()
