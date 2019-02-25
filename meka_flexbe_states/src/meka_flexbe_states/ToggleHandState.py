#!/usr/bin/env python

import rospy
import rospkg

from flexbe_core import Logger, EventState

from posture_execution.posture_execution import PostureExecution

class ToggleHandState(EventState):

    def __init__(self, hand, carrying, posture_path=''):

        super(ToggleHandState, self).__init__(outcomes=['success', 'failure'])

        self._rospack = rospkg.RosPack()
        self._meka_posture = PostureExecution('posture_exec')

        self._carrying = carrying

        self._meka_posture._posture_when_done = ''
        self._hand_name = hand + '_hand'

        if posture_path == '':
            posture_path = self._rospack.get_path('posture_execution') + '/config/postures_nonverbal_hand_over.yml'

        Logger.loginfo('Loading postures from: %s' % posture_path)
        self._meka_posture.load_postures(posture_path)

    def toggle_hand(self):
        return self._meka_posture.execute(self._hand_name, 'open' if self._carrying else 'close')

    def execute(self, d):
        if self._meka_posture.all_done:
            return 'success'

    def on_enter(self, d):
        self.toggle_hand()
        self._carrying = not self._carrying
