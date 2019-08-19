#!/usr/bin/env python
from flexbe_core import EventState

class CarryingState(EventState):

    ''' Does nothing. '''

    def __init__(self, carrying):
        
        self._carrying = carrying

        super(CarryingState, self).__init__(outcomes=['carrying', 'notCarrying'])

    def execute(self, d):
        if(self._carrying):
            return 'carrying'
        else:
            return 'notCarrying'
