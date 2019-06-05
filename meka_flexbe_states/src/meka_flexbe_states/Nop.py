#!/usr/bin/env python
from flexbe_core import EventState

class Nop(EventState):

    ''' Does nothing. '''

    def __init__(self):

        super(Nop, self).__init__(outcomes=['done'])

    def execute(self, d):
        return 'done'
