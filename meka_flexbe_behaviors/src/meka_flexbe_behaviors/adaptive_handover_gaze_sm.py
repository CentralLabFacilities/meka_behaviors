#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from meka_flexbe_states.HandoverAdaptionReset import HandoverAdaptionReset
from meka_flexbe_states.WaitForForce import WaitForForceState
from flexbe_states.wait_state import WaitState
from meka_flexbe_behaviors.deterministicgazehandhandface_sm import DeterministicGazeHandHandFaceSM
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
from flexbe_states.check_condition_state import CheckConditionState
from meka_flexbe_states.ChooseRandomSuccessor import ChooseNextStateRandomly
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
from flexbe_states.subscriber_state import SubscriberState
from meka_flexbe_states.GroupToPosture import GroupToPosture
from meka_flexbe_behaviors.deterministicgazeiotpface_sm import DeterministicGazeiOTPFaceSM
from meka_flexbe_states.GazeAt import GazeAtTarget
from meka_flexbe_states.StopHandTracking import StopHandTracking
from meka_flexbe_states.AdjustTorso import AdjustTorso
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.ToggleHandState import ToggleHandState
from meka_flexbe_states.RemoteRecord import RemoteRecord
from meka_flexbe_states.WaitForDoldButton import WaitForDoldButton
from meka_flexbe_states.RemoteRecordStop import RemoteRecordStop
from flexbe_states.flexible_check_condition_state import FlexibleCheckConditionState
from flexbe_states.calculation_state import CalculationState
from meka_flexbe_states.SetKey import SetKey
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
		self.add_parameter('stop_dist', 1.1)
		self.add_parameter('participant_id', 1)
		self.add_parameter('num_runs', 1)

		# references to used behaviors
		self.add_behavior(DeterministicGazeHandHandFaceSM, 'WaitContactGazeatHumanHand/DeterministicGazeHandHandFace')
		self.add_behavior(DeterministicGazeHandHandFaceSM, 'AdaptionGazeAtHuman/Gaze/gazeUntiliOTP/DeterministicGazeHandHandFace')
		self.add_behavior(DeterministicGazeiOTPFaceSM, 'AdaptionGazeAtHuman/Gaze/DeterministicGazeiOTPFace')
		self.add_behavior(DeterministicGazeHandHandFaceSM, 'PushInHandGazeatHuman/DeterministicGazeHandHandFace')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:35 y:461
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.carrying = self.carrying
		_state_machine.userdata.num_runs = self.num_runs
		_state_machine.userdata.run_count = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_graspgive_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['carrying'], output_keys=['carrying'])

		with _sm_graspgive_0:
			# x:51 y:71
			OperatableStateMachine.add('GraspGive',
										ToggleHandState(hand=self.hand, carrying=self.carrying, posture_path=''),
										transitions={'success': 'finished', 'failure': 'ReopenOnFail'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:128 y:194
			OperatableStateMachine.add('ReopenOnFail',
										GroupToPosture(hand='right', group='hand', posture='little_closed', posture_path='', non_blocking=False),
										transitions={'success': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		# x:30 y:365
		_sm_gazerobotneutral_1 = OperatableStateMachine(outcomes=['finished'])

		with _sm_gazerobotneutral_1:
			# x:43 y:73
			OperatableStateMachine.add('robotHand',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'finished', 'done': 'neutral'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:226 y:99
			OperatableStateMachine.add('neutral',
										GazeAtTarget(target='neutral', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'finished'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_checkmoving_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_checkmoving_2:
			# x:64 y:103
			OperatableStateMachine.add('getAdaptionFeedback',
										SubscriberState(topic='/do_adaption/feedback', blocking=True, clear=True),
										transitions={'received': 'isMoving', 'unavailable': 'getAdaptionFeedback'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:249 y:197
			OperatableStateMachine.add('isMoving',
										CheckConditionState(predicate=lambda x: x.feedback.state != 0),
										transitions={'true': 'finished', 'false': 'getAdaptionFeedback'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'message'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_gazeuntiliotp_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('DeterministicGazeHandHandFace', 'done')]),
										('finished', [('checkMoving', 'finished')]),
										('failed', [('checkMoving', 'failed')])
										])

		with _sm_gazeuntiliotp_3:
			# x:52 y:144
			OperatableStateMachine.add('DeterministicGazeHandHandFace',
										self.use_behavior(DeterministicGazeHandHandFaceSM, 'AdaptionGazeAtHuman/Gaze/gazeUntiliOTP/DeterministicGazeHandHandFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'carrying_in': 'carrying'})

			# x:289 y:71
			OperatableStateMachine.add('checkMoving',
										_sm_checkmoving_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_gaze_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['carrying'])

		with _sm_gaze_4:
			# x:167 y:75
			OperatableStateMachine.add('gazeUntiliOTP',
										_sm_gazeuntiliotp_3,
										transitions={'finished': 'DeterministicGazeiOTPFace', 'failed': 'DeterministicGazeiOTPFace'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:167 y:234
			OperatableStateMachine.add('DeterministicGazeiOTPFace',
										self.use_behavior(DeterministicGazeiOTPFaceSM, 'AdaptionGazeAtHuman/Gaze/DeterministicGazeiOTPFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit})


		# x:30 y:365, x:785 y:69
		_sm_openlittleafterwait_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['carrying'])

		with _sm_openlittleafterwait_5:
			# x:233 y:38
			OperatableStateMachine.add('checkCarrying',
										CheckConditionState(predicate=lambda x: x),
										transitions={'true': 'KeepParaRunning', 'false': 'getAdaptionFeedback'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'carrying'})

			# x:79 y:153
			OperatableStateMachine.add('KeepParaRunning',
										WaitState(wait_time=999),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:511 y:46
			OperatableStateMachine.add('getAdaptionFeedback',
										SubscriberState(topic='/do_adaption/feedback', blocking=True, clear=True),
										transitions={'received': 'checkIsAdapting', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:490 y:164
			OperatableStateMachine.add('checkIsAdapting',
										CheckConditionState(predicate=lambda x: x.feedback.state != 0),
										transitions={'true': 'CloseLittle', 'false': 'getAdaptionFeedback'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:293 y:191
			OperatableStateMachine.add('CloseLittle',
										GroupToPosture(hand='right', group='hand', posture='little_closed', posture_path='', non_blocking=True),
										transitions={'success': 'KeepParaRunning', 'failure': 'KeepParaRunning'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		# x:106 y:527, x:501 y:561
		_sm_disabledalternatinggazeheadrobhand_6 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_disabledalternatinggazeheadrobhand_6:
			# x:30 y:40
			OperatableStateMachine.add('wait4ever',
										WaitState(wait_time=300000),
										transitions={'done': 'getAdaptionFeedback'},
										autonomy={'done': Autonomy.Off})

			# x:399 y:23
			OperatableStateMachine.add('isMoving',
										CheckConditionState(predicate=lambda x: x.feedback.state != 0),
										transitions={'true': 'chooseGazeMoving', 'false': 'chooseGazeNotMoving'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'message'})

			# x:567 y:233
			OperatableStateMachine.add('chooseGazeMoving',
										ChooseNextStateRandomly(number_of_options=4),
										transitions={'1': 'gazeAtHumanHand', '2': 'gazeAtRobotHand', '3': 'gazeAtiOTP', '4': 'gazeAtFace', '5': 'failed'},
										autonomy={'1': Autonomy.Off, '2': Autonomy.Off, '3': Autonomy.Off, '4': Autonomy.Off, '5': Autonomy.Off})

			# x:682 y:81
			OperatableStateMachine.add('chooseGazeNotMoving',
										ChooseNextStateRandomly(number_of_options=3),
										transitions={'1': 'gazeAtHumanHand', '2': 'gazeAtRobotHand', '3': 'gazeAtFace', '4': 'failed', '5': 'failed'},
										autonomy={'1': Autonomy.Off, '2': Autonomy.Off, '3': Autonomy.Off, '4': Autonomy.Off, '5': Autonomy.Off})

			# x:45 y:226
			OperatableStateMachine.add('gazeAtHumanHand',
										GazeAtTargetReturn(target='right_hand', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'getAdaptionFeedback', 'done': 'getAdaptionFeedback'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:126 y:356
			OperatableStateMachine.add('gazeAtRobotHand',
										GazeAtTargetReturn(target='robot_right_hand', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'getAdaptionFeedback', 'done': 'getAdaptionFeedback'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:359 y:341
			OperatableStateMachine.add('gazeAtiOTP',
										GazeAtTargetReturn(target='iotp', duration_type='short', use_timeout=True),
										transitions={'target_not_found': 'getAdaptionFeedback', 'done': 'getAdaptionFeedback'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:207 y:467
			OperatableStateMachine.add('gazeAtFace',
										GazeAtTargetReturn(target='face', duration_type='long', use_timeout=True),
										transitions={'target_not_found': 'getAdaptionFeedback', 'done': 'getAdaptionFeedback'},
										autonomy={'target_not_found': Autonomy.Off, 'done': Autonomy.Off})

			# x:191 y:76
			OperatableStateMachine.add('getAdaptionFeedback',
										SubscriberState(topic='/do_adaption/feedback', blocking=True, clear=True),
										transitions={'received': 'isMoving', 'unavailable': 'getAdaptionFeedback'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365
		_sm_pushinhandgazeathuman_7 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('HandoverAdaptionExecPush', 'succeeded')]),
										('failed', [('HandoverAdaptionExecPush', 'error')]),
										('finished', [('DeterministicGazeHandHandFace', 'done')]),
										('finished', [('ForceDuringMovement', 'success')]),
										('failed', [('ForceDuringMovement', 'failure')])
										])

		with _sm_pushinhandgazeathuman_7:
			# x:47 y:142
			OperatableStateMachine.add('HandoverAdaptionExecPush',
										HandoverAdaptionExec(command=0, topic='/do_adaption', reality_damp=0.4, terminate_dist_override=0.07, terminate_timeout_override=1.0, fixed_orientation=True, terminate=True, use_reference_trajectory=False),
										transitions={'succeeded': 'finished', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:412 y:138
			OperatableStateMachine.add('DeterministicGazeHandHandFace',
										self.use_behavior(DeterministicGazeHandHandFaceSM, 'PushInHandGazeatHuman/DeterministicGazeHandHandFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'carrying_in': 'carrying'})

			# x:216 y:142
			OperatableStateMachine.add('ForceDuringMovement',
										WaitForForceState(hand=self.hand, force_threshold=3.5),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_togglehandgaze_8 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], output_keys=['carrying'], conditions=[
										('failed', [('robotHand', 'target_not_found')]),
										('finished', [('GraspGive', 'finished')]),
										('failed', [('GraspGive', 'failed')])
										])

		with _sm_togglehandgaze_8:
			# x:46 y:133
			OperatableStateMachine.add('GraspGive',
										_sm_graspgive_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:333 y:163
			OperatableStateMachine.add('robotHand',
										GazeAtTarget(target='robot_right_hand', duration_type='short', use_timeout=False),
										transitions={'target_not_found': 'failed'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365
		_sm_adjusttorso_9 = ConcurrencyContainer(outcomes=['finished'], conditions=[
										('finished', [('adjustTorso', 'done')]),
										('finished', [('gaze@face', 'target_not_found')])
										])

		with _sm_adjusttorso_9:
			# x:59 y:159
			OperatableStateMachine.add('adjustTorso',
										AdjustTorso(person_stop_dist=self.stop_dist, with_j1=True, rate=10),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:204 y:161
			OperatableStateMachine.add('gaze@face',
										GazeAtTarget(target='face', duration_type='long', use_timeout=False),
										transitions={'target_not_found': 'finished'},
										autonomy={'target_not_found': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_retreatgaze_10 = ConcurrencyContainer(outcomes=['finished'], conditions=[
										('finished', [('GazeRobotNeutral', 'finished')]),
										('finished', [('gohome', 'succeeded')]),
										('finished', [('gohome', 'error')]),
										('finished', [('StopHandTracking', 'done')])
										])

		with _sm_retreatgaze_10:
			# x:98 y:111
			OperatableStateMachine.add('gohome',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'succeeded': 'finished', 'error': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:324 y:109
			OperatableStateMachine.add('GazeRobotNeutral',
										_sm_gazerobotneutral_1,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:584 y:115
			OperatableStateMachine.add('StopHandTracking',
										StopHandTracking(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365, x:730 y:365, x:830 y:365, x:930 y:365
		_sm_adaptiongazeathuman_11 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('handoverAdaption', 'succeeded')]),
										('failed', [('handoverAdaption', 'error')]),
										('finished', [('DISABLEDAlternatingGazeHeadRobHand', 'finished')]),
										('failed', [('DISABLEDAlternatingGazeHeadRobHand', 'failed')]),
										('finished', [('OpenLittleAfterWait', 'finished')]),
										('failed', [('OpenLittleAfterWait', 'failed')]),
										('finished', [('Gaze', 'finished')]),
										('failed', [('Gaze', 'failed')])
										])

		with _sm_adaptiongazeathuman_11:
			# x:70 y:155
			OperatableStateMachine.add('handoverAdaption',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.7, terminate_dist_override=0.07, terminate_timeout_override=10.0, fixed_orientation=True, terminate=True, use_reference_trajectory=True),
										transitions={'succeeded': 'finished', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:286 y:45
			OperatableStateMachine.add('DISABLEDAlternatingGazeHeadRobHand',
										_sm_disabledalternatinggazeheadrobhand_6,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:454 y:177
			OperatableStateMachine.add('OpenLittleAfterWait',
										_sm_openlittleafterwait_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:634 y:64
			OperatableStateMachine.add('Gaze',
										_sm_gaze_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_waitcontactgazeathumanhand_12 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('waitforContact', 'success')]),
										('failed', [('waitforContact', 'failure')]),
										('failed', [('ForceTimeout', 'done')]),
										('finished', [('DeterministicGazeHandHandFace', 'done')])
										])

		with _sm_waitcontactgazeathumanhand_12:
			# x:48 y:85
			OperatableStateMachine.add('waitforContact',
										WaitForForceState(hand=self.hand, force_threshold=2.5),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:164 y:85
			OperatableStateMachine.add('ForceTimeout',
										WaitState(wait_time=3),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:370 y:157
			OperatableStateMachine.add('DeterministicGazeHandHandFace',
										self.use_behavior(DeterministicGazeHandHandFaceSM, 'WaitContactGazeatHumanHand/DeterministicGazeHandHandFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'carrying_in': 'carrying'})



		with _state_machine:
			# x:70 y:24
			OperatableStateMachine.add('resetAdaption',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'succeeded': 'start_button', 'error': 'resetAdaption'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:535 y:360
			OperatableStateMachine.add('WaitContactGazeatHumanHand',
										_sm_waitcontactgazeathumanhand_12,
										transitions={'finished': 'ToggleHandGaze', 'failed': 'PushInHandGazeatHuman'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:574 y:209
			OperatableStateMachine.add('AdaptionGazeAtHuman',
										_sm_adaptiongazeathuman_11,
										transitions={'finished': 'WaitContactGazeatHumanHand', 'failed': 'AdjustTorso'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:289 y:559
			OperatableStateMachine.add('RetreatGaze',
										_sm_retreatgaze_10,
										transitions={'finished': 'stopRecording'},
										autonomy={'finished': Autonomy.Inherit})

			# x:509 y:41
			OperatableStateMachine.add('AdjustTorso',
										_sm_adjusttorso_9,
										transitions={'finished': 'InitHandTracking'},
										autonomy={'finished': Autonomy.Inherit})

			# x:677 y:6
			OperatableStateMachine.add('InitHandTracking',
										InitHandTracking(),
										transitions={'done': 'right_arm_hand_over_ready'},
										autonomy={'done': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:339 y:429
			OperatableStateMachine.add('ToggleHandGaze',
										_sm_togglehandgaze_8,
										transitions={'finished': 'RetreatGaze', 'failed': 'PushInHandGazeatHuman'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:662 y:91
			OperatableStateMachine.add('right_arm_hand_over_ready',
										GroupToPosture(hand='right', group='arm', posture='right_arm_hand_over_ready', posture_path='', non_blocking=True),
										transitions={'success': 'AdaptionGazeAtHuman', 'failure': 'AdaptionGazeAtHuman'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:547 y:492
			OperatableStateMachine.add('PushInHandGazeatHuman',
										_sm_pushinhandgazeathuman_7,
										transitions={'finished': 'ToggleHandGaze', 'failed': 'WaitContactGazeatHumanHand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:356 y:51
			OperatableStateMachine.add('startRecording',
										RemoteRecord(topic='/meka/rosbagremote/record/named', pid=self.participant_id),
										transitions={'done': 'AdjustTorso'},
										autonomy={'done': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:46 y:117
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'zeroCounter'},
										autonomy={'done': Autonomy.Off})

			# x:93 y:539
			OperatableStateMachine.add('stopRecording',
										RemoteRecordStop(topic='/meka/rosbagremote/record/named'),
										transitions={'done': 'incrementRunCount'},
										autonomy={'done': Autonomy.Off})

			# x:137 y:247
			OperatableStateMachine.add('checkNumRuns',
										FlexibleCheckConditionState(predicate=lambda x: 2*x[0] <= x[1], input_keys=['num_runs', 'run_count']),
										transitions={'true': 'start_button', 'false': 'startRecording'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'num_runs': 'num_runs', 'run_count': 'run_count'})

			# x:77 y:403
			OperatableStateMachine.add('incrementRunCount',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'checkNumRuns'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'run_count', 'output_value': 'run_count'})

			# x:210 y:86
			OperatableStateMachine.add('zeroCounter',
										SetKey(value=0),
										transitions={'done': 'startRecording'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_key': 'run_count'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
