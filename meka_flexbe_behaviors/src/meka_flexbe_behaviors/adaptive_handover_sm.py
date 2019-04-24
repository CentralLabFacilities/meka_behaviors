#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.AdjustTorso import AdjustTorso
from meka_flexbe_states.HandoverAdaptionInit import HandoverAdaptionInit
from meka_flexbe_states.ToggleHandState import ToggleHandState
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.WaitForDoldButton import WaitForDoldButton
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
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
		self.add_parameter('stop_dist', 1.3)

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
			# x:38 y:192
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'adjust_torso', 'failure': 'finished'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:795 y:28
			OperatableStateMachine.add('adaption_init',
										HandoverAdaptionInit(topic='/hace/people', x_min=0, x_max=0.5, y_min=-0.75, y_max=0.05, z_min=-0.25, z_max=0.55),
										transitions={'right_hand_in_ws': 'adaption_exec', 'left_hand_in_ws': 'adaption_exec', 'error': 'failed'},
										autonomy={'right_hand_in_ws': Autonomy.Off, 'left_hand_in_ws': Autonomy.Off, 'error': Autonomy.Off})

			# x:390 y:360
			OperatableStateMachine.add('toggle_hand',
										ToggleHandState(hand=self.hand, carrying=self.carrying, posture_path=''),
										transitions={'success': 'start_button', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:317 y:33
			OperatableStateMachine.add('init_handtracking',
										InitHandTracking(),
										transitions={'done': 'adaption_exec'},
										autonomy={'done': Autonomy.Off})

			# x:432 y:151
			OperatableStateMachine.add('adaption_exec',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False, terminate=True, dynamic_orientation=True),
										transitions={'stopped': 'failed', 'succeeded': 'toggle_hand', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:99 y:33
			OperatableStateMachine.add('adjust_torso',
										AdjustTorso(person_stop_dist=self.stop_dist, with_j1=True, rate=10),
										transitions={'done': 'init_handtracking'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
