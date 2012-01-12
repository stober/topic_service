#! /usr/bin/env python
"""
Author: Jeremy M. Stober
Program: TEST_TOPIC_SERVER.PY
Date: Thursday, January 12 2012
Description: Test topic server. See test.launch.
"""

import roslib; roslib.load_manifest('topic_server')
import rospy
from std_msgs.msg import String
from topic_server.srv import StringSrv
from topic_server import TopicServer

def transform(msg):
    return msg.data

ts = TopicServer('test', 'chatter','chat_server', String, StringSrv, transform)
ts.spin()



