#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String

class RemoteRecord(EventState):

    ''' Starts rosbag recording. '''

    def __init__(self, topic='/meka/rosbagremote/record/named', pid=1):
        self._count = 1
        self._pid = pid
        self._topic = topic
        self._pub = ProxyPublisher({topic: String})

        super(RemoteRecord, self).__init__(outcomes=['done'], input_keys=['carrying'])

    def execute(self, d):
        carry = 'CARRY' if d.carrying else 'NONCARRY'
        name = str(self._pid) + '_' + str(self._count) + '_' + carry

        Logger.loginfo('Recording bag ' + name)

        self._pub.publish(self._topic, name + ':start')
        self._count += 1

        return 'done'
