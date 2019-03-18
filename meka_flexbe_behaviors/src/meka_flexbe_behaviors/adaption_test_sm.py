#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.WaitForDoldButton import WaitForDoldButton
from meka_flexbe_states.HandoverAdaptionInit import HandoverAdaptionInit
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 04 2019
@author: me
'''
class adaption_testSM(Behavior):
	'''
	test for behavior adaption
	'''


	def __init__(self):
		super(adaption_testSM, self).__init__()
		self.name = 'adaption_test'

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
			# x:334 y:56
			OperatableStateMachine.add('adapt_init',
										HandoverAdaptionInit(topic='/hace/people', x_min=0, x_max=0.5, y_min=-0.75, y_max=0.05, z_min=-0.25, z_max=0.55),
										transitions={'right_hand_in_ws': 'adapt_button_wait', 'left_hand_in_ws': 'failed', 'error': 'failed'},
										autonomy={'right_hand_in_ws': Autonomy.Off, 'left_hand_in_ws': Autonomy.Off, 'error': Autonomy.Off})

			# x:633 y:148
			OperatableStateMachine.add('adapt_exec',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False, terminate=True, dynamic_orientation=True),
										transitions={'stopped': 'failed', 'succeeded': 'adapt_reset', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:387 y:307
			OperatableStateMachine.add('adapt_reset',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False, terminate=True, dynamic_orientation=True),
										transitions={'stopped': 'failed', 'succeeded': 'finished', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:568 y:47
			OperatableStateMachine.add('adapt_button_wait',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'adapt_exec', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
