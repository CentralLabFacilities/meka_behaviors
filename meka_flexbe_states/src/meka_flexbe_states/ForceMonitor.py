#!/usr/bin/env python

import numpy as np
import rospy
import rospkg

from flexbe_core import Logger, EventState
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import WrenchStamped

'''
author: MZB & llach
'''

class ForceMonitor(EventState):
    '''
    Monitors Force (derivative).
    '''

    def __init__(self, force_threshold=40.0, topic='/force_helper'):
        '''Constructor'''
        super(ForceMonitor, self).__init__(outcomes=['success'])

        self._threshold = force_threshold
        self._topic = topic
        self._sub = ProxySubscriberCached({self._topic: WrenchStamped})


    def execute(self, d):
        '''Execute this state'''
        msg = self._sub.get_last_msg(self._topic)
        current_force = np.array([np.clip(msg.wrench.force.x, -99, 0), np.clip(msg.wrench.force.y, 0, 99), msg.wrench.force.z*0.5])
        
        #y contains gravity here
        #current_force = msg.wrench.force.y
        force_norm = 0.5*np.linalg.norm(current_force)

        if force_norm > self._threshold:
            Logger.loginfo('got force: force_norm %r _threshold %r' %(force_norm , self._threshold))
            return 'success'

