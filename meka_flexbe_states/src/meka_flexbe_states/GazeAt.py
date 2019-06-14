#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from gaze_relay.msg import GazeRelayTarget

import random
import rospy


class GazeAtTarget(EventState):

    ''' Calls gaze relay. '''

    def __init__(self, target='face', duration_type='long', use_timeout=True):
        assert duration_type == 'long' or duration_type == 'short', 'unknown duration type: ' + str(duration_type)

        super(GazeAtTarget, self).__init__(outcomes = ['target_not_found'])

        self._gaze_topic = '/gaze_relay/target'
        self._pub = ProxyPublisher({self._gaze_topic: GazeRelayTarget})
        self._target = target
        self._use_timeout = use_timeout

        if duration_type == 'long':
            self._mean_length =  1.4
            self._standard_deviation = 1.3
            self._min_length = 0.5
            self._max_length = 2.5
        else:
            self._mean_length = 0.77
            self._standard_deviation = 0.5
            self._min_length = 0.27
            self._max_length = 1.27

        self._duration = 0
        self._start_time = rospy.get_rostime()

        self.target_map = {
            'face':         GazeRelayTarget.FACE,
            'neutral':      GazeRelayTarget.NEUTRAL,
            'right_hand':   GazeRelayTarget.RIGHT_HAND,
            'left_hand':    GazeRelayTarget.LEFT_HAND,
            'torso':        GazeRelayTarget.TORSO,
        }

    def on_enter(self, _):

        '''Upon entering the state, save the start time and calculate the duration.'''

        self._start_time = rospy.get_rostime()

        self._duration = random.gauss(self._mean_length, self._standard_deviation)
        self._duration = max(self._duration, self._min_length)
        self._duration = min(self._duration, self._max_length)

        Logger.loginfo('Gazing Duration: %r' % self._duration)

    def execute(self, _):

        if self._target not in self.target_map.keys():
            Logger.logerr('Target ' + str(self._target) + ' not found')
            return 'target_not_found'

        elapsed_time = rospy.get_rostime() - self._start_time
        if (elapsed_time.to_sec() < self._duration) or not self._use_timeout:
            Logger.loginfo('gazing at ' + str(self._target))
            tar = GazeRelayTarget()
            tar.person_id = 0
            tar.gaze_target = self.target_map[self._target]
            self._pub.publish(self._gaze_topic, tar)
        else:
            Logger.loginfo('gazing at ' + str(self._target) + ' reached timeout! publishing neutral target.')
            tar = GazeRelayTarget()
            tar.person_id = 0
            tar.gaze_target = self.target_map['neutral']
            self._pub.publish(self._gaze_topic, tar)
