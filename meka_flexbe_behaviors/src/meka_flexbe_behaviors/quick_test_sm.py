#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.GroupToPosture import GroupToPosture
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Apr 24 2019
@author: me
'''
class quick_testSM(Behavior):
	'''
	this statemachine is meant for quickly debugging new states
	'''


	def __init__(self):
		super(quick_testSM, self).__init__()
		self.name = 'quick_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:103 y:40
			OperatableStateMachine.add('asd',
										GroupToPosture(hand='right', group='arm', posture='right_arm_hand_over_approach_high', posture_path=''),
										transitions={'success': 'w78', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:380 y:39
			OperatableStateMachine.add('w78',
										WaitState(wait_time=4),
										transitions={'done': 'grou'},
										autonomy={'done': Autonomy.Off})

			# x:317 y:163
			OperatableStateMachine.add('grou',
										GroupToPosture(hand='right', group='arm', posture='right_arm_hand_over_retreat', posture_path=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
