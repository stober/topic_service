#!/usr/bin/env python

# A service to listen and return data from a topic.

import roslib; roslib.load_manifest('topic_service')
import rospy
import threading

class Service2Topic:

    def __init__(self, node_name, topic_name, server_name, topic_type, server_type, hz, transform, default = None):
        rospy.init_node(node_name)
        self.rate = rospy.Rate(hz)
        self.params = default
        self.transform = transform

        self.server_name = server_name
        self.service = rospy.ServiceProxy(server_name, server_type)
        self.topic = rospy.Publisher(topic_name, topic_type)
        self.topic_name = topic_name

    def check_params(self):
        if rospy.has_param(self.topic_name + "_parameters"):
            self.params = rospy.get_param(self.topic_name + "_parameters")

    def spin(self):
        rospy.wait_for_service(self.server_name)
        while not rospy.is_shutdown():
            self.check_params() # look for changes to the parameters
            data = self.service(**self.params)
            self.topic.publish(self.transform(data))
            self.rate.sleep()

class Topic2Service:

    def __init__(self, node_name, topic_name, server_name, topic_type, server_type, transform):
        rospy.init_node(node_name)
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
