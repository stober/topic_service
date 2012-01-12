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
from topic_service.srv import StringSrv
from topic_service import TopicService

def transform(msg):
    return msg.data

ts = TopicService('test', 'chatter','chat_service', String, StringSrv, transform)
ts.spin()



