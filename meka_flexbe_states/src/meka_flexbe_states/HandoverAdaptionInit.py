#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

import rospy
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from hace_msgs.msg import MinimalHumans


class HandoverAdaptionInit(EventState):

    ''' Returns which hand of the handover partner is detected in the robot's workspace. '''

    def __init__(self, topic='/hace/people', x_min=0, x_max=0.5, y_min=-0.75, y_max=0.05, z_min=-0.25, z_max=0.55):

        super(HandoverAdaptionInit, self).__init__(outcomes=['right_hand_in_ws', 'left_hand_in_ws', 'error'])

        rospy.loginfo('Starting initiation state.')

        # define workspace
        self._x_min = x_min
        self._x_max = x_max
        self._y_min = y_min
        self._y_max = y_max
        self._z_min = z_min
        self._z_max = z_max

        self._ws_transformed = False
        self._tf = TransformListener()
        self._topic = topic

        self._person = None
        self._persons = None

        self._sub = ProxySubscriberCached({self._topic: MinimalHumans})

        self._error = False

    def execute(self, userdata):

        self._persons = self._sub.get_last_msg(self._topic)

        if self._persons is not None:

            self._person = self._persons.humans[0]

            if self._person.right_hand is not None:

                hh_position = PointStamped()
                hh_position.header.frame_id = 'camera_depth_frame'
                hh_position.point.x = self._person.right_hand.position.x
                hh_position.point.y = self._person.right_hand.position.y
                hh_position.point.z = self._person.right_hand.position.z

                t_hh_position = self._tf.transformPoint('/upper', hh_position)

                if t_hh_position.point.x > self._x_min and t_hh_position.point.x < self._x_max and \
                   t_hh_position.point.y > self._y_min and t_hh_position.point.y < self._y_max and \
                   t_hh_position.point.z > self._z_min and t_hh_position.point.z < self._z_max:
                    return 'right_hand_in_ws'

            elif self._person.left_hand is not None:
                left_hand_pos_x = self._person.left_hand.position.x
                left_hand_pos_y = self._person.left_hand.position.y
                left_hand_pos_z = self._person.left_hand.position.z

                if left_hand_pos_x > self._x_min and left_hand_pos_x < self._x_max and \
                   left_hand_pos_y > self._y_min and left_hand_pos_y < self._y_max and \
                   left_hand_pos_z > self._z_min and left_hand_pos_z < self._z_max:
                    return 'left_hand_in_ws'

        if self._error:
            return 'error'

    def on_enter(self, userdata):

        self._persons = self._sub.get_last_msg(self._topic)

        if self._persons is not None:
            self._person = self._persons.humans[0]

        self._error = False

    def on_exit(self, userdata):

        rospy.loginfo('Exit initiation of adaption.')
