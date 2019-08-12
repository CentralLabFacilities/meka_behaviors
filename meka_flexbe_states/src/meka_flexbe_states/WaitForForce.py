#!/usr/bin/env python

import numpy
import rospy
import rospkg

from flexbe_core import Logger, EventState
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import WrenchStamped

'''
author: jpohlmeyer, decluttered a little by llach
'''


class WaitForForceState(EventState):
    '''
    Implements a state waits for force.

    -- posture_path The path to the posture config.

    <= success      The movement was successfull.

    <= failure      The movement was not successfull.

    '''

    def __init__(self, hand, force_threshold=1.5):
        '''Constructor'''
        super(WaitForForceState, self).__init__(outcomes=['success', 'failure'])
        self._action_name = rospy.get_name()
        self._prefix = "meka_roscontrol"
        self._client = {}
        self._movement_finished = {}
        self.force_variation = {}
        self.force_bias = {}
        self.previous_force = {}
        self._r = rospy.Rate(100)
        self._threshold = force_threshold

        # store latest planning scene
        self.current_planning_scene = None

        self.force_variation['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_variation['right_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_bias['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_bias['right_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.previous_force['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.previous_force['right_arm'] = numpy.array([0.0, 0.0, 0.0])

        self._current_val = 0

        self._group = hand + '_arm'

        self.sub_left = ProxySubscriberCached({'/meka_ros_pub/m3loadx6_ma30_l0/wrench': WrenchStamped})
        self.sub_right = ProxySubscriberCached({'/meka_ros_pub/m3loadx6_ma29_l0/wrench': WrenchStamped})

    def handle_left(self, msg):
        if msg == None:
            return
        current_force = numpy.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_variation['left_arm'] = (current_force - self.previous_force['left_arm'])
        self.previous_force['left_arm'] = current_force

    def handle_right(self, msg):
        if msg == None:
            return
        current_force = numpy.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.force_variation['right_arm'] = (current_force - self.previous_force['right_arm'])
        self.previous_force['right_arm'] = current_force

    def wait_for_force(self, threshold, group_name, timeout=None):
        success = True
        # wait for condition
        if timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration(timeout)

        current_val = 0.0

        self.force_bias[group_name] = self.previous_force[group_name];
        rospy.loginfo('waiting for force with bias %f %f %f', self.force_bias[group_name][0],
                      self.force_bias[group_name][1], self.force_bias[group_name][2])
        N = 7
        while current_val < threshold:

            current_val = ((N - 1) * current_val + numpy.linalg.norm(
                self.previous_force[group_name] - self.force_bias[group_name])) / N
            rospy.logdebug('current_val: %f', current_val)

            if timeout is not None:
                if rospy.Time.now() > timeout_time:
                    rospy.loginfo("timeout waiting for force")
                    success = False
                    break

            self._r.sleep()

        rospy.loginfo("wait_for_force ended with: current_val: %f threshold: %f", current_val, threshold)
        return success

    def check_force(self, threshold, group_name, timeout=None):
        success = False
        N = 7

        self.handle_left(self.sub_left.get_last_msg('/meka_ros_pub/m3loadx6_ma30_l0/wrench'))
        self.handle_right(self.sub_right.get_last_msg('/meka_ros_pub/m3loadx6_ma29_l0/wrench'))

        if self._current_val < threshold:

            self._current_val = ((N - 1) * self._current_val + numpy.linalg.norm(
                self.previous_force[group_name] - self.force_bias[group_name])) / N
            rospy.logdebug('current_val: %f', self._current_val)
            if timeout is not None:
                if rospy.get_rostime().to_sec() > self._start_time.to_sec() + timeout:
                    rospy.loginfo("timeout waiting for force")
                    success = True
        else:
            success = True

        rospy.loginfo("wait_for_force ended with: current_val: %f threshold: %f", self._current_val, threshold)
        return success

    # end methods from handover node

    def execute(self, d):
        '''Execute this state'''

        if self.check_force(threshold=self._threshold, group_name=self._group, timeout=120.0):
            Logger.loginfo('got force!')
            return 'success'

    def on_enter(self, d):
        '''Upon entering the state check the value of the given topic'''

        Logger.loginfo('waiting for force at {}'.format(self._group))

        self.force_variation['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_variation['right_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_bias['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.force_bias['right_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.previous_force['left_arm'] = numpy.array([0.0, 0.0, 0.0])
        self.previous_force['right_arm'] = numpy.array([0.0, 0.0, 0.0])

        self._current_val = 0

        self._start_time = rospy.get_rostime()

        self.handle_left(self.sub_left.get_last_msg('/meka_ros_pub/m3loadx6_ma30_l0/wrench'))
        self.handle_right(self.sub_right.get_last_msg('/meka_ros_pub/m3loadx6_ma29_l0/wrench'))

        self.force_bias[self._group] = self.previous_force[self._group];

