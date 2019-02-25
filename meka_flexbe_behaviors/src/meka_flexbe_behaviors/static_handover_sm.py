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
from flexbe_states.log_state import LogState
from meka_flexbe_states.GroupToPosture import GroupToPosture
from meka_flexbe_states.ToggleHandState import ToggleHandState
from meka_flexbe_states.WaitForForce import WaitForForceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Feb 19 2019
@author: llach
'''
class static_handoverSM(Behavior):
	'''
	handover without adaption or gaze
	'''


	def __init__(self):
		super(static_handoverSM, self).__init__()
		self.name = 'static_handover'

		# parameters of this behavior
		self.add_parameter('hand', 'right')
		self.add_parameter('carrying', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:810 y:362, x:47 y:397
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:35
			OperatableStateMachine.add('wait4dold',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'approach', 'failure': 'fail'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:782 y:196
			OperatableStateMachine.add('done',
										LogState(text='done!', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:63 y:259
			OperatableStateMachine.add('fail',
										LogState(text='something went wrong', severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:576 y:227
			OperatableStateMachine.add('retreat',
										GroupToPosture(hand=self.hand, group='arm', posture=self.hand + '_arm_hand_over_retreat', posture_path=''),
										transitions={'success': 'wait4dold', 'failure': 'fail'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:582 y:36
			OperatableStateMachine.add('toggle_hand',
										ToggleHandState(hand=self.hand, carrying=self.carrying, posture_path=''),
										transitions={'success': 'retreat', 'failure': 'fail'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:220 y:33
			OperatableStateMachine.add('approach',
										GroupToPosture(hand=self.hand, group='arm', posture=self.hand + '_arm_hand_over_approach_low', posture_path=''),
										transitions={'success': 'wait4force', 'failure': 'fail'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:407 y:21
			OperatableStateMachine.add('wait4force',
										WaitForForceState(hand=self.hand),
										transitions={'success': 'toggle_hand', 'failure': 'fail'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
