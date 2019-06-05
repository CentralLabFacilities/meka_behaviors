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
from meka_flexbe_states.AdjustTorso import AdjustTorso
from meka_flexbe_states.GazeAt import GazeAtTarget
from meka_flexbe_states.Nop import Nop
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 04 2019
@author: me
'''
class adaptive_handover_gazeSM(Behavior):
	'''
	handover with trajectory adaption and gazing at things
	'''


	def __init__(self):
		super(adaptive_handover_gazeSM, self).__init__()
		self.name = 'adaptive_handover_gaze'

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

		# x:30 y:365, x:130 y:365
		_sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_container_0:
			# x:166 y:43
			OperatableStateMachine.add('gaceface',
										GazeAtTarget(target='face', wait=None),
										transitions={'done': 'nop', 'target_not_found': 'failed'},
										autonomy={'done': Autonomy.Off, 'target_not_found': Autonomy.Off})

			# x:376 y:66
			OperatableStateMachine.add('nop',
										Nop(),
										transitions={'done': 'gaceface'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_inittorso_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('adjustTorso', 'done')]),
										('finished', [('Container', 'finished')]),
										('finished', [('Container', 'failed')])
										])

		with _sm_inittorso_1:
			# x:119 y:83
			OperatableStateMachine.add('adjustTorso',
										AdjustTorso(person_stop_dist=0.5, with_j1=False, rate=10),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:306 y:54
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:73 y:50
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'InitTorso', 'failure': 'finished'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:259 y:45
			OperatableStateMachine.add('InitTorso',
										_sm_inittorso_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
