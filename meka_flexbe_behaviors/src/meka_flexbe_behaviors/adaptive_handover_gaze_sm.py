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
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
from flexbe_states.wait_state import WaitState
from meka_flexbe_states.ToggleHandState import ToggleHandState
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
		self.add_parameter('stop_dist', 0.7)

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

		# x:130 y:365
		_sm_lookathandoverspotafterxsec_0 = OperatableStateMachine(outcomes=['failed'])

		with _sm_lookathandoverspotafterxsec_0:
			# x:114 y:61
			OperatableStateMachine.add('waitXsec',
										WaitState(wait_time=5),
										transitions={'done': 'gazeAtHumanHand'},
										autonomy={'done': Autonomy.Off})

			# x:285 y:68
			OperatableStateMachine.add('gazeAtHumanHand',
										GazeAtTarget(target='right_hand', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_startadaptionandlookathumanhandtwice_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('failed', [('AdaptionStart', 'stopped')]),
										('finished', [('AdaptionStart', 'succeeded')]),
										('failed', [('gaze@HumanHand', 'target_not_found')]),
										('failed', [('LookAtHandoverSpotAfterXsec', 'failed')]),
										('failed', [('AdaptionStart', 'error')])
										])

		with _sm_startadaptionandlookathumanhandtwice_1:
			# x:278 y:54
			OperatableStateMachine.add('AdaptionStart',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False, terminate=True, dynamic_orientation=True),
										transitions={'stopped': 'failed', 'succeeded': 'finished', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:585 y:173
			OperatableStateMachine.add('LookAtHandoverSpotAfterXsec',
										_sm_lookathandoverspotafterxsec_0,
										transitions={'failed': 'failed'},
										autonomy={'failed': Autonomy.Inherit})

			# x:339 y:186
			OperatableStateMachine.add('gaze@HumanHand',
										GazeAtTarget(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_inittorso_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('adjustTorso', 'done')]),
										('failed', [('gaze@face', 'target_not_found')])
										])

		with _sm_inittorso_2:
			# x:59 y:159
			OperatableStateMachine.add('adjustTorso',
										AdjustTorso(person_stop_dist=self.stop_dist, with_j1=True, rate=10),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:362 y:93
			OperatableStateMachine.add('gaze@face',
										GazeAtTarget(target='face', duration_type='long', use_timeout=False),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})



		with _state_machine:
			# x:73 y:50
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'InitTorso', 'failure': 'finished'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:259 y:45
			OperatableStateMachine.add('InitTorso',
										_sm_inittorso_2,
										transitions={'finished': 'InitHandTracking', 'failed': 'InitHandTracking'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:476 y:42
			OperatableStateMachine.add('InitHandTracking',
										InitHandTracking(),
										transitions={'done': 'StartAdaptionAndLookAtHumanHandTwice'},
										autonomy={'done': Autonomy.Off})

			# x:673 y:38
			OperatableStateMachine.add('StartAdaptionAndLookAtHumanHandTwice',
										_sm_startadaptionandlookathumanhandtwice_1,
										transitions={'finished': 'ToggleHand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:549 y:224
			OperatableStateMachine.add('ToggleHand',
										ToggleHandState(hand='right_hand', carrying=self.carrying, posture_path=''),
										transitions={'success': 'start_button', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
