#!/usr/bin/env python

import rospy

from flexbe_core import Logger, EventState
from flexbe_core.proxy import ProxyServiceCaller

from handtracking.srv import Handtracking, HandtrackingRequest

class StopHandTracking(EventState):

    def __init__(self):

        super(StopHandTracking, self).__init__(outcomes=['done'])

        self.TRACKING_TOPIC = '/handtracking_server/stop'
        self._srv = ProxyServiceCaller({
            self.TRACKING_TOPIC: Handtracking
        })

        Logger.loginfo('waiting for handtracking service server')

        while not self._srv.is_available(self.TRACKING_TOPIC):
            rospy.Rate(10).sleep()

        Logger.loginfo('service found.')

    def execute(self, d):
        Logger.loginfo('stopping handtracking ...')
        req = HandtrackingRequest()
        #req.carrying = True
        self._srv.call(self.TRACKING_TOPIC, req)
        Logger.loginfo('done')
        return 'done'
