#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.WaitForPerson import WaitForPersonState
from meka_flexbe_states.HandoverAdaptionInit import HandoverAdaptionInit
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
from meka_flexbe_states.ToggleHandState import ToggleHandState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 04 2019
@author: me
'''
class adaptive_handoverSM(Behavior):
	'''
	handover with trajectory adaption
	'''


	def __init__(self):
		super(adaptive_handoverSM, self).__init__()
		self.name = 'adaptive_handover'

		# parameters of this behavior
		self.add_parameter('hand', 'right')
		self.add_parameter('carrying', False)

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
			# x:99 y:75
			OperatableStateMachine.add('wait5person',
										WaitForPersonState(target_sub_topic='/hace/people'),
										transitions={'done': 'adapt_init', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:383 y:25
			OperatableStateMachine.add('adapt_init',
										HandoverAdaptionInit(topic='/my/topic', x_min=0, x_max=0, y_min=0, y_max=0, z_min=0, z_max=0),
										transitions={'right_hand_in_ws': 'adaption_exec', 'left_hand_in_ws': 'adapt_init', 'error': 'failed'},
										autonomy={'right_hand_in_ws': Autonomy.Off, 'left_hand_in_ws': Autonomy.Off, 'error': Autonomy.Off})

			# x:405 y:190
			OperatableStateMachine.add('adaption_exec',
										HandoverAdaptionExec(command=0, topic=0, reality_damp=0, fixed_orientation=0, terminate=0, dynamic_orientation=0),
										transitions={'stopped': 'failed', 'succeeded': 'toggle_gripper', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:423 y:345
			OperatableStateMachine.add('toggle_gripper',
										ToggleHandState(hand=self.hand, carrying=self.carrying, posture_path=''),
										transitions={'success': 'wait5person', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
