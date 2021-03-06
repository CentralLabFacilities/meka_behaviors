#!/usr/bin/env python

import rospy
import rospkg

from flexbe_core import Logger, EventState

from posture_execution.posture_execution import PostureExecution

class GroupToPosture(EventState):

    def __init__(self, hand='right', group='hand', posture='handover', posture_path='', non_blocking=True):
        super(GroupToPosture, self).__init__(outcomes=['success', 'failure'])

        self._rospack = rospkg.RosPack()
        self._meka_posture = PostureExecution('posture_exec')

        self._posture = posture

        self._meka_posture._posture_when_done = ''
        self._hand_name = hand + '_' + group if group in ['arm', 'hand'] else group # torso and head dont have handedness

        if posture_path == '':
            posture_path = self._rospack.get_path('posture_execution') + '/config/postures_nonverbal_hand_over.yml'

        self._non_blocking = non_blocking

        Logger.loginfo('Loading postures from: %s' % posture_path)
        self._meka_posture.load_postures(posture_path)

    def execute(self, d):
        if self._meka_posture.all_done:
            return 'success'
        elif self._non_blocking:
            return 'success'

    def on_enter(self, d):
        Logger.loginfo('executing posture ' + self._posture + ' for ' + self._hand_name)
        self._meka_posture.execute(self._hand_name, self._posture)
