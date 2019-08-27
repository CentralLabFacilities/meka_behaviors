#!/usr/bin/env python

import rospy
import random
from flexbe_core import EventState, Logger

'''
author: jpohlmeyer
'''

class ChooseNextStateRandomly(EventState):
    '''
    Implements a state that gives a random outcome between 1 and number_of_options

    -- number_of_options 	int	    The number of possible outcomes (currently maximal 5).

    <= 1                            Outcome 1

    <= 2                            Outcome 2

    <= 3                            Outcome 3

    <= 4                            Outcome 4

    <= 5                            Outcome 5

    '''
    def __init__(self, number_of_options):
        '''Constructor'''
        super(ChooseNextStateRandomly, self).__init__(outcomes=['1','2','3','4','5'])

        self._number_of_options = number_of_options
        self._outcome = 1

    def execute(self, userdata):
        '''Execute this state'''

        return str(self._outcome)

    def on_enter(self, userdata):
        '''Upon entering the state choose a random number between 0 and number_of_options'''

        self._outcome = random.randint(1,self._number_of_options)

        Logger.loginfo('Chosen Number: %r' % self._outcome)