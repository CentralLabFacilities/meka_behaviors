#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 23 2019
@author: me
'''
class DeterministicGazeiOTPFaceSM(Behavior):
	'''
	gazing
	'''


	def __init__(self):
		super(DeterministicGazeiOTPFaceSM, self).__init__()
		self.name = 'DeterministicGazeiOTPFace'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['done'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:172 y:89
			OperatableStateMachine.add('gazeAtiOTP',
										GazeAtTargetReturn(target='iotp', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'gazeAtFace', 'done': 'gazeAtFace'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:518 y:96
			OperatableStateMachine.add('gazeAtFace',
										GazeAtTargetReturn(target='face', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'gazeAtiOTP', 'done': 'gazeAtiOTP'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
