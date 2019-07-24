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
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
from meka_flexbe_states.GroupToPosture import GroupToPosture
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
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


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_retreatgaze_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('RetreatPosture', 'success')]),
										('failed', [('RetreatPosture', 'failure')]),
										('finished', [('GazeRobotNeutral', 'finished')]),
										('failed', [('GazeRobotNeutral', 'failed')])
										])

		with _sm_retreatgaze_3:
			# x:73 y:154
			OperatableStateMachine.add('RetreatPosture',
										GroupToPosture(hand='right', group='hand', posture='handover', posture_path=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:324 y:109
			OperatableStateMachine.add('GazeRobotNeutral',
										_sm_gazerobotneutral_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_togglehandgaze_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('ToggleHand', 'success')]),
										('failed', [('ToggleHand', 'failure')]),
										('failed', [('robotHand', 'target_not_found')])
										])

		with _sm_togglehandgaze_4:
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
		_sm_adaptiongazeathuman_5 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('handoverAdaption', 'stopped')]),
										('finished', [('handoverAdaption', 'succeeded')]),
										('failed', [('handoverAdaption', 'error')]),
										('finished', [('AlternatingGazeHeadRobHand', 'finished')]),
										('failed', [('AlternatingGazeHeadRobHand', 'failed')])
										])

		with _sm_adaptiongazeathuman_5:
			# x:83 y:168
			OperatableStateMachine.add('handoverAdaption',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.5, fixed_orientation=False, terminate=True, dynamic_orientation=True),
										transitions={'stopped': 'finished', 'succeeded': 'finished', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:439 y:41
			OperatableStateMachine.add('AlternatingGazeHeadRobHand',
										_sm_alternatinggazeheadrobhand_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_prehandovergazeathumanhand_6 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('preHandover', 'success')]),
										('failed', [('preHandover', 'failure')]),
										('finished', [('AlternatingGazeHumRobHand', 'finished')]),
										('failed', [('AlternatingGazeHumRobHand', 'failed')])
										])

		with _sm_prehandovergazeathumanhand_6:
			# x:273 y:40
			OperatableStateMachine.add('AlternatingGazeHumRobHand',
										_sm_alternatinggazehumrobhand_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:75 y:188
			OperatableStateMachine.add('preHandover',
										GroupToPosture(hand='right', group='arm', posture='right_arm_hand_over_approach_low', posture_path=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_inittorso_7 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('adjustTorso', 'done')]),
										('failed', [('gaze@face', 'target_not_found')])
										])

		with _sm_inittorso_7:
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

			# x:291 y:45
			OperatableStateMachine.add('InitTorso',
										_sm_inittorso_7,
										transitions={'finished': 'InitHandTracking', 'failed': 'InitHandTracking'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:476 y:42
			OperatableStateMachine.add('InitHandTracking',
										InitHandTracking(),
										transitions={'done': 'PreHandoverGazeatHumanHand'},
										autonomy={'done': Autonomy.Off})

			# x:629 y:162
			OperatableStateMachine.add('PreHandoverGazeatHumanHand',
										_sm_prehandovergazeathumanhand_6,
										transitions={'finished': 'AdaptionGazeAtHuman', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:417 y:208
			OperatableStateMachine.add('AdaptionGazeAtHuman',
										_sm_adaptiongazeathuman_5,
										transitions={'finished': 'ToggleHandGaze', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:168 y:281
			OperatableStateMachine.add('ToggleHandGaze',
										_sm_togglehandgaze_4,
										transitions={'finished': 'RetreatGaze', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:179 y:169
			OperatableStateMachine.add('RetreatGaze',
										_sm_retreatgaze_3,
										transitions={'finished': 'start_button', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
