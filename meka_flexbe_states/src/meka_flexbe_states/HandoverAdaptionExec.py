#!/usr/bin/env python
from flexbe_core import EventState, Logger

import actionlib
import rospy

from jacobian_control.msg import DoAdaptionAction, DoAdaptionGoal, DoAdaptionResult, DoAdaptionFeedback


class HandoverAdaptionExec(EventState):

    ''' Calls jacobian-control node for adaption. '''

    def __init__(self, command=0, topic='/do_adaption', reality_damp=0.3, terminate_dist_override=0.0, terminate_timeout_override=0.0, fixed_orientation=True,
                 terminate=True, use_reference_trajectory=True, joint_speed_limit=0.1):

        super(HandoverAdaptionExec, self).__init__(outcomes = ['succeeded', 'error'])

        self._topic = topic     #'do_adaption'
        self._command = command
        self._reality_damp = reality_damp       # 0.5
        self._terminate_dist_override = terminate_dist_override       # 0.5
        self._terminate_timeout_override = terminate_timeout_override       # 0.5
        self._fixed_orientation = fixed_orientation
        self._terminate = terminate
        self._joint_speed_limit = joint_speed_limit

        self._use_reference_trajectory =use_reference_trajectory

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
        if self._client.get_state() is 4:       # aborted
            rospy.logwarn(self._client.get_state())
            return 'error'

    def on_enter(self, userdata):
        # Create the goal.
        goal = DoAdaptionGoal()
        goal.command = DoAdaptionGoal.COMMAND_ADAPT
        goal.reality_damp = self._reality_damp
        goal.fixed_orientation = self._fixed_orientation
        goal.terminate = self._terminate
        goal.terminate_dist_override = self._terminate_dist_override
        goal.terminate_timeout_override = self._terminate_timeout_override
        goal.joint_speed_limit = self._joint_speed_limit

        goal.use_reference_trajectory = self._use_reference_trajectory
        Logger.loginfo('sending goal: %s' %str(goal))
        self._error = False # make sure to reset the error state since a previous state execution might have failed

        try:
            self._client.send_goal(goal)
        except Exception as e:
            self._error = True

    def on_exit(self, userdata):
        self._client.cancel_all_goals()
        rospy.loginfo('Exit adaption.')
