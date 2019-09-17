#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.WaitForForce import WaitForForceState
from flexbe_states.wait_state import WaitState
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

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed', 'timeout'], conditions=[
										('finished', [('wff', 'success')]),
										('failed', [('wff', 'failure')]),
										('timeout', [('w8', 'done')])
										])

		with _sm_container_0:
			# x:102 y:87
			OperatableStateMachine.add('wff',
										WaitForForceState(hand='right', force_threshold=1.5),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:254 y:86
			OperatableStateMachine.add('w8',
										WaitState(wait_time=3),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:71 y:110
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed', 'timeout': 'log'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'timeout': Autonomy.Inherit})

			# x:313 y:108
			OperatableStateMachine.add('log',
										LogState(text='timeout', severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
