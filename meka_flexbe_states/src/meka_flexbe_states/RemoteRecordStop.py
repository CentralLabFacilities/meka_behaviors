#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String

class RemoteRecordStop(EventState):

    ''' Stops rosbag recording. '''

    def __init__(self, topic='/meka/rosbagremote/record/named'):
        self._topic = topic
        self._pub = ProxyPublisher({topic: String})

        super(RemoteRecordStop, self).__init__(outcomes=['done'])

    def execute(self, d):
        Logger.loginfo('Stopping rosbag recording')
        self._pub.publish(self._topic, ':stop')

        return 'done'
