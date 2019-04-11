#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from geometry_msgs.msg import PointStamped
from people_msgs.msg import People
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

import rospy
import tf

import numpy as np


class AdjustTorso(EventState):

    def __init__(self, person_stop_dist=0.5, with_j1=False, rate=10):

        super(AdjustTorso, self).__init__(outcomes = ['done'])

        self.with_j1 = bool(with_j1)
        self.rate = rate
        self.person_stop_dist = person_stop_dist

        self.MAX_VEL = 1.0
        self.MIN_TRAJ_DUR = 0.5
        self.SPEED_SCALE = 0.5
        self.JOINT_NAMES = ['torso_j0', 'torso_j1']
        self.TARGET_FRAME = 'panplate'

        self.LIMITS = {
            self.JOINT_NAMES[0]: [-0.75, 0.75],
            self.JOINT_NAMES[1]: [-0.1, 0.07],
        }

        self.j1_done = False
        self.people_pos = None
        self.js_pos = None
        self.js_pos_des = None

        self.t = tf.TransformListener(True, rospy.Duration(10))

        self.PEOPLE_TOPIC = '/people_tracker/people'
        self.TORSO_STATE_TOPIC = '/meka_roscontrol/torso_position_trajectory_controller/state'
        self.TORSO_COMMAND_TOPIC = '/meka_roscontrol/torso_position_trajectory_controller/command'
        self.FACES_TOPIC = '/openface2/faces'


        sub_dict = {
            self.TORSO_STATE_TOPIC: JointTrajectoryControllerState,
            self.PEOPLE_TOPIC: People
        }

        if self.with_j1:
            from openface2_ros_msgs.msg import Faces
            sub_dict.update({
                self.FACES_TOPIC: Faces
            })

        self._subs = ProxySubscriberCached(sub_dict)
        self._pub = ProxyPublisher({self.TORSO_COMMAND_TOPIC: JointTrajectory})

        frames = []
        Logger.loginfo('waiting for transforms')
        while frames == []:
            frames = self.t.getFrameStrings()

    def execute(self, d):
        torso_state, people, faces = None, None, None
        ra = rospy.Rate(self.rate)

        Logger.loginfo('waiting for torso state and people')

        got_it = False
        # wait until we have data
        while not got_it:
            torso_state = self._subs.get_last_msg(self.TORSO_STATE_TOPIC)
            people = self._subs.get_last_msg(self.PEOPLE_TOPIC)

            if self.with_j1:
                faces = self._subs.get_last_msg(self.FACES_TOPIC)
                if faces is None:
                    Logger.loginfo('no faces')
                    continue
                if faces.count < 1:
                    Logger.loginfo('no faces')
                    continue

            if torso_state is None:
                Logger.loginfo('no torso state')
            elif people is None:
                Logger.loginfo('no people')
            elif len(people.people) < 1:
                Logger.loginfo('no people')
            else:
                got_it = True
            ra.sleep()

        Logger.loginfo('Got initial people and torso state.')

        while True:

            # first, get new data if not None
            ts = self._subs.get_last_msg(self.TORSO_STATE_TOPIC)
            pe = self._subs.get_last_msg(self.PEOPLE_TOPIC)

            if self.with_j1:
                fa = self._subs.get_last_msg(self.FACES_TOPIC)
                if fa is not None:
                    if fa.count > 0:
                        faces = fa

            if ts is not None:
                torso_state = ts
            if pe is not None:
                if len(pe.people) > 0:
                    people = pe

            ps = PointStamped()
            ps.header = people.header
            ps.header.stamp = rospy.Time(0)
            ps.point = people.people[0].position

            trans = self.t.transformPoint(self.TARGET_FRAME, ps)
            person_pos = np.arctan2(trans.point.y, trans.point.x)
            person_dist = np.linalg.norm([trans.point.x,trans.point.y])

            js_pos = torso_state.actual.positions
            js_pos_des = torso_state.desired.positions

            dur = []

            traj = JointTrajectory()
            traj.joint_names = self.JOINT_NAMES
            point = JointTrajectoryPoint()

            pos = js_pos[0]
            cmd = np.clip(person_pos, self.LIMITS[self.JOINT_NAMES[0]][0], self.LIMITS[self.JOINT_NAMES[0]][1])

            dur.append(max(abs(cmd - pos) / self.MAX_VEL, self.MIN_TRAJ_DUR))

            if self.with_j1 and not self.j1_done:
                if faces is None:
                    print('faces none')
                    continue
                elif faces.count < 1:
                    print('no faces')
                    continue

                f_pos = faces.faces[0].head_pose

                ps = PointStamped()
                ps.header = faces.header
                ps.header.stamp = rospy.Time(0)
                ps.point = f_pos.position

                trans = self.t.transformPoint(self.TARGET_FRAME, ps)

                if trans is None:
                    print('trans none')
                    continue

                faces_pos = -np.arctan2(trans.point.z - 0.7, trans.point.x) / 2

                j1cmd = np.clip(faces_pos, self.LIMITS[self.JOINT_NAMES[1]][0], self.LIMITS[self.JOINT_NAMES[1]][1])
                j1pos = js_pos[1]
                dur.append(max(abs(j1cmd - j1pos) / self.MAX_VEL, self.MIN_TRAJ_DUR))
                point.positions = [cmd, j1cmd]
            else:
                point.positions = [cmd, js_pos_des[1]]

            point.time_from_start = rospy.Duration(max(dur) / self.SPEED_SCALE)
            traj.points.append(point)
            Logger.loginfo(point.positions)

            self._pub.publish(self.TORSO_COMMAND_TOPIC, traj)

            if float(person_dist) < float(self.person_stop_dist):
                Logger.loginfo('Person {} nearer than {}. Stopping.'.format(person_dist, self.person_stop_dist))
                return 'done'
            else:
                continue
