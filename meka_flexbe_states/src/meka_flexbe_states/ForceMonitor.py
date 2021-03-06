#!/usr/bin/env python

import numpy as np
import rospy
import rospkg

from flexbe_core import Logger, EventState
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32

'''
author: MZB & llach
'''

class ForceMonitor(EventState):
    '''
    Monitors Force (derivative).
    '''

    def __init__(self, force_threshold=0.5, force_topic='/force_helper/result'):
        '''Constructor'''
        super(ForceMonitor, self).__init__(outcomes=['success'], input_keys=['carrying'])

        self._threshold = force_threshold
        self._force_topic = force_topic
        self._sub = ProxySubscriberCached({self._force_topic: Float32})


    def execute(self, d):
        '''Execute this state'''
        force_msg = self._sub.get_last_msg(self._force_topic)


        if force_msg is None:
            return

        #current_force = np.array([np.clip(force_msg.wrench.force.x, -99, 0), np.clip(force_msg.wrench.force.y, 0, 99), force_msg.wrench.force.z*0.5])
        force_norm = force_msg.data

        current_threshold = self._threshold # * (0.5+current_acc)

        if d.carrying:
            current_threshold *= 1.5

        #y contains gravity here
        #current_force = force_msg.wrench.force.y
        #force_norm = np.linalg.norm(current_force)
        #Logger.loginfo('time: %r  got force: force_norm %r _threshold %r adapted thresh %r' % (rospy.Time.now().nsecs/1000000, force_norm, self._threshold, current_threshold))

        if force_norm > current_threshold:
            Logger.loghint('got force: force_norm %r _threshold %r adapted thresh %r' %(force_norm , self._threshold, current_threshold))
            return 'success'

