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


def transform(msg):
    return msg.data

ts = Topic2Service('test', 'chatter','chat_service', String, StringSrv, transform)
ts.spin()
