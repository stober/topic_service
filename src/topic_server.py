#!/usr/bin/env python

# A service to listen and return data from a topic.

import roslib; roslib.load_manifest('topic_server')
import rospy
import threading
import copy

# TODO: add a clock argument to service (to get topic data that is current?)

class TopicServer:

    def __init__(self, node_name, topic_name, server_name, topic_type, server_type):
        rospy.init_node(topic_name)
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        self.data = None
        self.thread = threading.Thread(target=self.topicListener, args=(topic_name, topic_type))
        self.thread.start()
        self.acquired = False

        self.service = rospy.Service(server_name, server_type, self.returnTopic)
        self.server_type = server_type

    def topicListener(self, topic_name, topic_type):
        rospy.Subscriber(topic_name, topic_type, self.topicCallback)
        rospy.spin()

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
            self.data = copy.deepcopy(msg) # alt: self.data = msg
            self.acquired = True
            self.cv.notify()

        self.cv.release()

    def __del__(self):
        pass

    def returnTopic(self, req):
        self.cv.acquire()

        # Trigger a copy of topic data into the server thread.
        self.acquired = False
        while not self.acquired:
            self.cv.wait()

        self.cv.release()
        return self.data.data

if __name__ == "__main__":

    # Testing code assumes that "rosrun rospy_tutorials talker.py" has been called.
    from std_msgs.msg import String
    from topic_server.srv import StringSrv

    ts = TopicServer('test', 'chatter','chat_server', String, StringSrv)
