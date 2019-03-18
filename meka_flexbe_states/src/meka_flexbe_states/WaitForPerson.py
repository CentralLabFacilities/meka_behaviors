#!/usr/bin/env python

import rospy
import random
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from hace_msgs.msg import MinimalHumans

'''
author: jpohlmeyer
'''

class WaitForPersonState(EventState):
    '''
    Implements a state that returns true if there is a face published on the given topic.

    -- target_sub_topic     String  The topic for the coordinates where to gaze.

    <= done			                There is a person.

    <= failure		                Something bad happend.

    '''
    def __init__(self, target_sub_topic='/hace/people'):
        '''Constructor'''
        super(WaitForPersonState, self).__init__(outcomes=['done','failure'])
        self._target_sub_topic = target_sub_topic

        self._persons = None
        self._start_sec = 0
        self._start_nsec = 0

        self._sub = ProxySubscriberCached({self._target_sub_topic: MinimalHumans})


    def execute(self, userdata):
        '''Execute this state'''

        self._persons = self._sub.get_last_msg(self._target_sub_topic)
        if (self._persons != None and len(self._persons.humans) > 0):
            if self._persons.humans[0].header.stamp.secs > self._start_sec or \
                    (self._persons.humans[0].header.stamp.secs == self._start_sec and self._persons.humans[0].header.stamp.nsecs > self._start_nsec) :
                return 'done'


    def on_enter(self, userdata):
        '''Upon entering the state, save the start time and calculate the duration.'''

        self._persons = self._sub.get_last_msg(self._target_sub_topic)
        if (self._persons != None and len(self._persons.humans) > 0):
            self._start_sec = self._persons.humans[0].header.stamp.secs
            self._start_nsec = self._persons.humans[0].header.stamp.nsecs




