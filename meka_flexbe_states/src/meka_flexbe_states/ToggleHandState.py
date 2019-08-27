#!/usr/bin/env python

import rospy
import rospkg

from flexbe_core import Logger, EventState

from posture_execution.posture_execution import PostureExecution
from meka_stiffness_control.stiffness_control import MekaStiffnessControl

class ToggleHandState(EventState):

    def __init__(self, hand, carrying, posture_path=''):

        super(ToggleHandState, self).__init__(outcomes=['success', 'failure'], output_keys=['carrying'])

        self._rospack = rospkg.RosPack()
        self._meka_posture = PostureExecution('posture_exec')

        self._carrying = carrying

        self._meka_posture._posture_when_done = ''
        self._hand_name = hand + '_hand'

        if posture_path == '':
            posture_path = self._rospack.get_path('posture_execution') + '/config/postures_nonverbal_hand_over.yml'

        Logger.loginfo('Loading postures from: %s' % posture_path)
        self._meka_posture.load_postures(posture_path)
        self._stiffness_control = MekaStiffnessControl("stiffness_control")

    def toggle_hand(self):
        Logger.loginfo('toggling hand to ' + 'open' if self._carrying else 'close')
        return self._meka_posture.execute(self._hand_name, 'open' if self._carrying else 'close')

    def execute(self, d):    
        if self._meka_posture.all_done:

            #simply open, nothing can really fail
            if self._carrying:
                self._carrying = not self._carrying
                return 'success'
            else: 
                #closing failed, so we might have something in the hand now!
                if self._meka_posture.failed:
                    Logger.loginfo('posture exec finished with failed so there was an object')
                    self._carrying = not self._carrying
                    return 'success'
                #closed completely, so most likely nothing in hand
                else:
                    Logger.loginfo('posture exec finished with success, so hand completely closed without object')
                    return 'failure'

    def on_enter(self, d):
        if "left" in self._hand_name:
            j0 = "left_hand_j0"
            j1 = "left_hand_j1"
            j2 = "left_hand_j2"
            j3 = "left_hand_j3"
            j4 = "left_hand_j4"
        else:
            j0 = "right_hand_j0"
            j1 = "right_hand_j1"
            j2 = "right_hand_j2"
            j3 = "right_hand_j3"
            j4 = "right_hand_j4"
        
        if(self._carrying):
            self._stiffness_control.change_stiffness([j0, j1, j2, j3, j4], [1, 1, 1, 1, 1])
        else:
            self._stiffness_control.change_stiffness([j0, j1, j2, j3, j4], [0.75, 0.6, 0.5, 0.5, 0.7])
            Logger.loginfo("making hand soft for grasping");
    
        self.toggle_hand()

    def on_exit(self, d):
        d.carrying = self._carrying

