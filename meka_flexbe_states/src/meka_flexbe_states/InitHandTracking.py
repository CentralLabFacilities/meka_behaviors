#!/usr/bin/env python

import rospy

from flexbe_core import Logger, EventState
from flexbe_core.proxy import ProxyServiceCaller

from handtracking.srv import Handtracking, HandtrackingRequest

class InitHandTracking(EventState):

    def __init__(self, carrying):

        super(InitHandTracking, self).__init__(outcomes=['done'])
        
        self._carrying = carrying

        self.TRACKING_TOPIC = '/handtracking_server/init'
        self._srv = ProxyServiceCaller({
            self.TRACKING_TOPIC: Handtracking
        })

        Logger.loginfo('waiting for handtracking service server')

        while not self._srv.is_available(self.TRACKING_TOPIC):
            rospy.Rate(10).sleep()

        Logger.loginfo('service found.')

    def execute(self, d):
        Logger.loginfo('starting handtracking ...')
        req = HandtrackingRequest()
        req.carrying = self._carrying
        self._srv.call(self.TRACKING_TOPIC, req)
        Logger.loginfo('done')
        return 'done'
