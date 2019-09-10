#!/usr/bin/env python
from flexbe_core import EventState, Logger


class SetKey(EventState):

    ''' sets key to value. '''

    def __init__(self, value):
        super(SetKey, self).__init__(outcomes=['done'], output_keys=['output_key'])

        self.val = value
    def execute(self, data):
        data.output_key = self.val
        Logger.loginfo('Set data to %d' % (self.val))

        return 'done'