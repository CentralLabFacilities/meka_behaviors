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
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.WaitForForce import WaitForForceState
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
from flexbe_states.wait_state import WaitState
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
from meka_flexbe_states.ToggleHandState import ToggleHandState
from meka_flexbe_states.GazeAt import GazeAtTarget
from meka_flexbe_states.HandoverAdaptionReset import HandoverAdaptionReset
from meka_flexbe_states.AdjustTorso import AdjustTorso
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
		self.add_parameter('stop_dist', 1.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:479 y:473
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_gazerobotneutral_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_gazerobotneutral_0:
			# x:43 y:73
			OperatableStateMachine.add('robotHand',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'failed', 'done': 'neutral'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:226 y:99
			OperatableStateMachine.add('neutral',
										GazeAtTarget(target='neutral', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_alternatinggazeheadrobhand_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_alternatinggazeheadrobhand_1:
			# x:79 y:102
			OperatableStateMachine.add('HumanHead',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=False),
										transitions={'target_not_found': 'failed', 'done': 'robotHand'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:300 y:107
			OperatableStateMachine.add('robotHand',
										GazeAtTargetReturn(target='robot_right_han', duration_type='long', use_timeout=False),
										transitions={'target_not_found': 'failed', 'done': 'HumanHead'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_alternatinggazehumrobhand_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_alternatinggazehumrobhand_2:
			# x:157 y:87
			OperatableStateMachine.add('humanHand',
										GazeAtTargetReturn(target='right_hand', duration_type='short', use_timeout=False),
										transitions={'target_not_found': 'failed', 'done': 'robotHand'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:377 y:117
			OperatableStateMachine.add('robotHand',
										GazeAtTargetReturn(target='robot_right_hand', duration_type='short', use_timeout=False),
										transitions={'target_not_found': 'failed', 'done': 'humanHand'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_inittorso_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('adjustTorso', 'done')]),
										('failed', [('gaze@face', 'target_not_found')])
										])

		with _sm_inittorso_3:
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


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_retreatgaze_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('GazeRobotNeutral', 'finished')]),
										('failed', [('GazeRobotNeutral', 'failed')]),
										('finished', [('gohome', 'succeeded')]),
										('finished', [('gohome', 'stopped')]),
										('failed', [('gohome', 'error')])
										])

		with _sm_retreatgaze_4:
			# x:98 y:111
			OperatableStateMachine.add('gohome',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'stopped': 'finished', 'succeeded': 'finished', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:324 y:109
			OperatableStateMachine.add('GazeRobotNeutral',
										_sm_gazerobotneutral_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_togglehandgaze_5 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('ToggleHand', 'success')]),
										('failed', [('ToggleHand', 'failure')]),
										('failed', [('robotHand', 'target_not_found')])
										])

		with _sm_togglehandgaze_5:
			# x:113 y:90
			OperatableStateMachine.add('ToggleHand',
										ToggleHandState(hand=self.hand, carrying=self.carrying, posture_path=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:326 y:84
			OperatableStateMachine.add('robotHand',
										GazeAtTarget(target='robot_right_hand', duration_type='short', use_timeout=False),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_adaptiongazeathuman_6 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('handoverAdaption', 'stopped')]),
										('finished', [('handoverAdaption', 'succeeded')]),
										('failed', [('handoverAdaption', 'error')]),
										('finished', [('AlternatingGazeHeadRobHand', 'finished')]),
										('failed', [('AlternatingGazeHeadRobHand', 'failed')])
										])

		with _sm_adaptiongazeathuman_6:
			# x:83 y:168
			OperatableStateMachine.add('handoverAdaption',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, terminate_dist_override=0.0, fixed_orientation=True, terminate=True, use_reference_trajectory=True),
										transitions={'stopped': 'finished', 'succeeded': 'finished', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:439 y:41
			OperatableStateMachine.add('AlternatingGazeHeadRobHand',
										_sm_alternatinggazeheadrobhand_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_waitcontactgazeathumanhand_7 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('AlternatingGazeHumRobHand', 'finished')]),
										('failed', [('AlternatingGazeHumRobHand', 'failed')]),
										('finished', [('waitforContact', 'success')]),
										('failed', [('waitforContact', 'failure')]),
										('failed', [('ForceTimeout', 'done')])
										])

		with _sm_waitcontactgazeathumanhand_7:
			# x:48 y:85
			OperatableStateMachine.add('waitforContact',
										WaitForForceState(hand='right', force_threshold=2.5),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:273 y:40
			OperatableStateMachine.add('AlternatingGazeHumRobHand',
										_sm_alternatinggazehumrobhand_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:668 y:34
			OperatableStateMachine.add('ForceTimeout',
										WaitState(wait_time=3),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:48 y:37
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'resetAdaption', 'failure': 'finished'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:671 y:33
			OperatableStateMachine.add('InitHandTracking',
										InitHandTracking(),
										transitions={'done': 'AdaptionGazeAtHuman'},
										autonomy={'done': Autonomy.Off})

			# x:596 y:321
			OperatableStateMachine.add('WaitContactGazeatHumanHand',
										_sm_waitcontactgazeathumanhand_7,
										transitions={'finished': 'ToggleHandGaze', 'failed': 'PushInHand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:614 y:185
			OperatableStateMachine.add('AdaptionGazeAtHuman',
										_sm_adaptiongazeathuman_6,
										transitions={'finished': 'waitToSettle', 'failed': 'InitTorso'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:201 y:370
			OperatableStateMachine.add('ToggleHandGaze',
										_sm_togglehandgaze_5,
										transitions={'finished': 'RetreatGaze', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:179 y:169
			OperatableStateMachine.add('RetreatGaze',
										_sm_retreatgaze_4,
										transitions={'finished': 'start_button', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:247 y:31
			OperatableStateMachine.add('resetAdaption',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'stopped': 'failed', 'succeeded': 'InitTorso', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:639 y:526
			OperatableStateMachine.add('PushInHand',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.3, terminate_dist_override=0.05, fixed_orientation=True, terminate=True, use_reference_trajectory=False),
										transitions={'stopped': 'WaitContactGazeatHumanHand', 'succeeded': 'ToggleHandGaze', 'error': 'WaitContactGazeatHumanHand'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:458 y:14
			OperatableStateMachine.add('InitTorso',
										_sm_inittorso_3,
										transitions={'finished': 'InitHandTracking', 'failed': 'InitHandTracking'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:762 y:247
			OperatableStateMachine.add('waitToSettle',
										WaitState(wait_time=0.5),
										transitions={'done': 'WaitContactGazeatHumanHand'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
