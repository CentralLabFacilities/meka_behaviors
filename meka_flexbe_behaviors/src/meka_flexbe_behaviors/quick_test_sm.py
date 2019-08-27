#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
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
		_state_machine.userdata.carry = self.global_bool

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:168 y:38
			OperatableStateMachine.add('check',
										CheckConditionState(predicate=lambda x: x == True),
										transitions={'true': 'printtrue', 'false': 'printfalse'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'carry'})

			# x:30 y:213
			OperatableStateMachine.add('printtrue',
										LogState(text='bool was true', severity=Logger.REPORT_WARN),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:159 y:193
			OperatableStateMachine.add('printfalse',
										LogState(text='bool was false', severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
