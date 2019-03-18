#!/usr/bin/env python
from flexbe_core import EventState, Logger

import actionlib
import rospy

from jacobian_control.msg import DoAdaptionAction, DoAdaptionGoal, DoAdaptionResult, DoAdaptionFeedback


class HandoverAdaptionExec(EventState):

    ''' Calls jacobian-control node for adaption. '''

    def __init__(self, command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False,
                 terminate=True, dynamic_orientation=True):

        super(HandoverAdaptionExec, self).__init__(outcomes = ['stopped', 'succeeded', 'error'])

        self._topic = topic     #'do_adaption'
        self._command = command
        self._reality_damp = reality_damp       # 0.5
        self._fixed_orientation = fixed_orientation
        self._terminate = terminate
        self._dynamic_orientation = dynamic_orientation

        self._client = actionlib.SimpleActionClient(self._topic, DoAdaptionAction)
        Logger.loginfo('Waiting for adaption server ...')
        self._client.wait_for_server()
        Logger.loginfo('found adaption server!')

        self._error = False

    def execute(self, userdata):
        # Check if the client failed to send the goal.
        if self._error:
            return 'error'
        if self._client.get_state() is 3:       # succeeded
            rospy.logwarn(self._client.get_state())
            return 'succeeded'

    def on_enter(self, userdata):
        # Create the goal.
        goal = DoAdaptionGoal()
        goal.command = self._command
        goal.reality_damp = self._reality_damp
        goal.fixed_orientation = self._fixed_orientation
        goal.terminate = self._terminate
        goal.dynamic_orientation = self._dynamic_orientation

        self._error = False # make sure to reset the error state since a previous state execution might have failed

        try:
            self._client.send_goal(goal)
        except Exception as e:
            self._error = True

    def on_exit(self, userdata):
        rospy.loginfo('Exit adaption.')
