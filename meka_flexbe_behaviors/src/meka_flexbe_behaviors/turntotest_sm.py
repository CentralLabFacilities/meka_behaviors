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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 11 2019
@author: llach
'''
class TurnToTestSM(Behavior):
	'''
	testing new turn to skill
	'''


	def __init__(self):
		super(TurnToTestSM, self).__init__()
		self.name = 'TurnToTest'

		# parameters of this behavior
		self.add_parameter('person_stop_dist', 0.7)

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
			# x:146 y:49
			OperatableStateMachine.add('dold',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'torsoToPerson', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:371 y:51
			OperatableStateMachine.add('torsoToPerson',
										AdjustTorso(person_stop_dist=self.person_stop_dist, with_j1=True, rate=10),
										transitions={'done': 'dold'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
