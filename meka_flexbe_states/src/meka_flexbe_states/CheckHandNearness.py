#!/usr/bin/env python
import tf
import rospy

import numpy as np

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from collections import deque

class CheckHandNearness(EventState):
    '''
    Example for a state to demonstrate which functionality is available for state implementation.
    This example lets the behavior wait until the given target_time has passed since the behavior has been started.

    -- target_time 	float 	Time which needs to have passed since the behavior started.

    <= continue 			Given time has passed.
    <= failed 				Example for a failure outcome.

    '''

    def __init__(self, dist_threshold=0.05, n_steps=10, target_frame='handover_frame_right'):
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(CheckHandNearness, self).__init__(outcomes=['near'])

        self._n_steps = n_steps
        self._dist_threshold = dist_threshold
        self._frame = target_frame

        self._topic = '/otpprediction/handtracking/hand_marker'
        self._hand_sub = ProxySubscriberCached({self._topic: Marker})
        self._listener = tf.TransformListener(True, rospy.Duration(10))

        self._off_x = rospy.get_param('/jacobian_control/goal_offset_x', 0)
        self._off_y = rospy.get_param('/jacobian_control/goal_offset_y', 0)
        self._off_z = rospy.get_param('/jacobian_control/goal_offset_z', 0)

        Logger.loginfo('offsets: ' + str(self._off_x) + ' ' + str(self._off_y) + ' ' + str(self._off_z))

        self._dist_buf = deque(maxlen=self._n_steps)

    def execute(self, userdata):
        cur_pose = self._hand_sub.get_last_msg(self._topic)
        self._hand_sub.remove_last_msg(self._topic)

        if cur_pose is None:
            # print('current pose none')
            return

        if  rospy.Time.now() - cur_pose.header.stamp > rospy.Duration(0.5):
            print('got old msg')
            return

        ps = PoseStamped()
        ps.header = cur_pose.header
        ps.pose = cur_pose.pose

        try:
            dist_pose = self._listener.transformPose(self._frame, ps)
        except Exception:
            print('oh no:')
            return

        dist = np.linalg.norm(np.array([dist_pose.pose.position.x+self._off_x,
                                        dist_pose.pose.position.y+self._off_y,
                                        dist_pose.pose.position.z+self._off_z]))
        # print('dist:' + str(dist))

        self._dist_buf.appendleft(dist)

        if len(self._dist_buf) == self._n_steps and (np.array(self._dist_buf) < self._dist_threshold).all():
            Logger.loghint('hand nearness threshold reached!')
            return 'near'


