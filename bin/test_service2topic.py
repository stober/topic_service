#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TEST_TOPIC_SERVICE.PY
Date: Thursday, January 12 2012
Description: Test topic service. See test.launch.
"""

import roslib; roslib.load_manifest('topic_service')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
from rospy_tutorials.srv import AddTwoInts
from topic_service.srv import StringSrv
from topic_service import Topic2Service, Service2Topic


def transform(response):
    return response.sum

# From the command line:
# rosparam set /add_two_ints_topic_parameters '{"a":1,"b":2}'

request = {'a' : 3, 'b' : 4}
ts = Service2Topic('test','add_two_ints_topic', 'add_two_ints', Int64, AddTwoInts, 5, transform, default = request)
ts.spin()
