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
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 23 2019
@author: me
'''
class DeterministicGazeHandHandFaceSM(Behavior):
	'''
	gazing
	'''


	def __init__(self):
		super(DeterministicGazeHandHandFaceSM, self).__init__()
		self.name = 'DeterministicGazeHandHandFace'

		# parameters of this behavior
		self.add_parameter('carrying', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['done'], input_keys=['carrying_in'])
		_state_machine.userdata.carrying_in = self.carrying

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:213 y:80
			OperatableStateMachine.add('checkCarrying',
										CheckConditionState(predicate=lambda x: x),
										transitions={'true': 'gazeAtRobotHandCarry', 'false': 'gazeAtHumanHand'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'carrying_in'})

			# x:507 y:71
			OperatableStateMachine.add('gazeAtFace',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'gazeAtHumanHand', 'done': 'gazeAtHumanHand'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:778 y:69
			OperatableStateMachine.add('gazeAtRobotHand',
										GazeAtTargetReturn(target='robot_right_hand', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'gazeAtFace', 'done': 'gazeAtFace'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:755 y:214
			OperatableStateMachine.add('gazeAtHumanHand',
										GazeAtTargetReturn(target='right_hand', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'gazeAtRobotHand', 'done': 'gazeAtRobotHand'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:75 y:216
			OperatableStateMachine.add('gazeAtFaceCarry',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'gazeAtRobotHandCarry', 'done': 'gazeAtRobotHandCarry'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:365 y:403
			OperatableStateMachine.add('gazeAtHumanHandCarry',
										GazeAtTargetReturn(target='right_hand', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'gazeAtFaceCarry', 'done': 'gazeAtFaceCarry'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:718 y:318
			OperatableStateMachine.add('gazeAtRobotHandCarry',
										GazeAtTargetReturn(target='robot_right_hand', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'gazeAtHumanHandCarry', 'done': 'gazeAtHumanHandCarry'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
