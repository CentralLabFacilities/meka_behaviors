#!/usr/bin/env python

import rospy
import rospkg

from flexbe_core import Logger, EventState

from posture_execution.posture_execution import PostureExecution

class OpenHand(EventState):

    def __init__(self, posture_path, hand):

        super(OpenHand, self).__init__(outcomes=['success', 'failure'])

        self._rospack = rospkg.RosPack()
        self._meka_posture = PostureExecution('posture_exec')

        self._meka_posture._posture_when_done = ''
        self._hand = hand

        if posture_path == '':
            posture_path = self._rospack.get_path('posture_execution') + '/config/postures_nonverbal_hand_over.yml'

        Logger.loginfo('Loading postures from: %s' % posture_path)
        self._meka_posture.load_postures(posture_path)

    def open_hand(self):
        hand_name = self._hand + '_hand'

        if self._meka_posture.execute(hand_name, 'open'):
            return True
        else:
            return False

    def execute(self, d):
        if self._meka_posture.all_done:
            return 'success'

    def on_enter(self, d):
        self.open_hand()
