#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from gaze_relay.msg import GazeRelayTarget

import time


class GazeAtTarget(EventState):

    ''' Calls jacobian-control node for adaption. '''

    def __init__(self, target='face', wait=None):

        super(GazeAtTarget, self).__init__(outcomes = ['done', 'target_not_found'])

        self._pub = ProxyPublisher({'/gaze_relay/target': GazeRelayTarget})
        self._wait = wait
        self._target = target

        self.target_map = {
            'face':         GazeRelayTarget.FACE,
            'neutral':      GazeRelayTarget.NEUTRAL,
            'right_hand':   GazeRelayTarget.RIGHT_HAND,
            'left_hand':    GazeRelayTarget.LEFT_HAND,
            'torso':        GazeRelayTarget.TORSO,
        }

    def execute(self, d):

        Logger.loginfo('executing gaze at ' + str(self._target))

        if self._target not in self.target_map.keys():
            Logger.logerr('Target ' + str(self._target) + ' not found')
            return 'target_not_found'

        tar = GazeRelayTarget()
        tar.person_id = 0
        tar.gaze_target = self.target_map[self._target]

        if self._wait is not None:
            time.sleep(self._wait)

        Logger.loginfo('gazing done')

        return 'done'
