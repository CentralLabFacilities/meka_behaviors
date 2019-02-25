#!/usr/bin/env python

import rospy

from dold_msgs.msg import DoldStates, DoldState
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

'''
author: jpohlmeyer
'''

class WaitForDoldButton(EventState):

    '''

    Implements a state that returns done if the dold button is pressed.
    '''

    def __init__(self, dold_button_topic='/dold_driver/state'):
        super(WaitForDoldButton, self).__init__(outcomes=['done','failure'])

        self._dold_button_topic = dold_button_topic
        self._sub = ProxySubscriberCached({self._dold_button_topic: DoldStates})

    def on_enter(self, userdata):
        Logger.loginfo('waiting for right upper dold button ...')

    def execute(self, d):
        msg = self._sub.get_last_msg(self._dold_button_topic)

        if (msg != None):
            for event in msg.inputs:
                if event.type == DoldState.BUTTON and event.state == DoldState.PRESSED and event.name == 'B0':
                    Logger.loginfo('detected button press!')
                    return 'done'
