#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.CheckHandNearness import CheckHandNearness
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
		self.add_parameter('global_bool', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.carrying = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:142
			OperatableStateMachine.add('init',
										InitHandTracking(),
										transitions={'done': 'check_near'},
										autonomy={'done': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:314 y:170
			OperatableStateMachine.add('check_near',
										CheckHandNearness(dist_threshold=0.12, n_steps=10, target_frame='handover_frame_right'),
										transitions={'near': 'finished'},
										autonomy={'near': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
