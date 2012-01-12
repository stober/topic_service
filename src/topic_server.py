#!/usr/bin/env python

# A service to listen and return data from a topic.

import roslib; roslib.load_manifest('topic_server')
import rospy
import threading
import copy

# TODO: add a clock argument to service (to get topic data that is current?)

class TopicServer:

    def __init__(self, node_name, topic_name, server_name, topic_type, server_type, transform):
        rospy.init_node(topic_name)
        self.data = None
        self.acquired = False
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        rospy.Subscriber(topic_name, topic_type, self.topicCallback)

        self.service = rospy.Service(server_name, server_type, self.returnTopic)
        self.transform = transform

    def topicCallback(self, msg):
        """
        My guess is that in Python there is really no need for
        condition variables and a deep copy from topic listener thread
        local data into self.data. However, this seems like the safest
        thread safe way to examine the topic in the midst of a single
        server call without any real knowledge of how or in what
        context the topic callback is actually called.
        """

        self.cv.acquire()

        if self.acquired is False:
            self.data = msg
            self.acquired = True
            self.cv.notify()

        self.cv.release()

    def spin(self):
        rospy.spin()

    def returnTopic(self, req):
        self.cv.acquire()

        # Trigger a copy of topic data into the server thread.
        self.acquired = False
        while not self.acquired:
            self.cv.wait()

        self.cv.release()
        return self.transform(self.data)

if __name__ == "__main__":

    # Testing code assumes that "rosrun rospy_tutorials talker.py" has been called.
    from std_msgs.msg import String
    from topic_server.srv import StringSrv

    def transform(msg):
        return msg.data

    ts = TopicServer('test', 'chatter','chat_server', String, StringSrv, transform)
    ts.spin()
