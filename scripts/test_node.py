#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from roscpp.srv import GetLoggers, GetLoggersResponse, GetLoggersRequest
import sys


class Test(object):
    def __init__(self, suffix1, suffix2):
        self.suffix2 = suffix2
        self.r = rospy.Rate(1)
        self.sub = rospy.Subscriber(
            '/test_suite/header_' + str(suffix1), Header, self._callback)
        self.pub = rospy.Publisher(
            '/test_suite/text_' + str(suffix1), String, queue_size=1)
        self.pub2 = rospy.Publisher(
            '/test_suite/header_' + str(suffix2), Header, queue_size=1)
        self.serv = rospy.Service(
            '/test_suite/service_' + str(suffix1),
            GetLoggers, self._service_handler)

    def _publish(self):
        self.pub.publish("This is a string of text.")
        h = Header()
        self.pub2.publish(h)

    def _callback(self, header):
        print("got a callback")
        print header

    def _service_handler(self, req):
        print("Service is returning a value")
        return GetLoggersResponse()

    def _service_proxy(self):

        try:
            rospy.wait_for_service(
                'test_suite/service_' + str(self.suffix2), .5)
            proxy = rospy.ServiceProxy(
                '/test_suite/service_' + str(self.suffix2), GetLoggers)

            answer = proxy.call(GetLoggersRequest())
            print "Service result:"
            print answer
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e
        except rospy.exceptions.ROSException as e:
            if "timeout" not in str(e):
                raise e
            print "Service call failed: %s" % e

    def run(self):

        print "Running tests..."
        try:
            while True:
                self._publish()
                self._service_proxy()
                self.r.sleep()
        except KeyboardInterrupt:
            self.sub.unregister()
            print "Stopping test"


if __name__ == '__main__':
    rospy.init_node('testnode', anonymous=True)
    argv = rospy.myargv(sys.argv)
    t = Test(argv[1], argv[2])
    t.run()
