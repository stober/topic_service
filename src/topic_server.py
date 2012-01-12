#!/usr/bin/env python

# A service to listen and return data from a topic.

import roslib; roslib.load_manifest('topic_server')
import rospy
import threading
import copy

class TopicServer:

    def __init__(self, node_name, topic_name, server_name, topic_type, server_type, transform):
        rospy.init_node(topic_name)
        self.data = None
        self.acquired = False
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        rospy.Subscriber(topic_name, topic_type, self.topicCallback)

        # There doesn't seem to be a way to construct server_type from topic type dynamically. Sigh.
        self.service = rospy.Service(server_name, server_type, self.returnTopic)
        self.transform = transform

    def topicCallback(self, msg):
        self.cv.acquire()

        if self.acquired is False:
            self.data = msg
            self.acquired = True
            self.cv.notify()

        self.cv.release()

    def spin(self):
        rospy.spin()

    # Override if you want to make this more complex.
    # E.g. Wait a certain amount of clock time.

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
