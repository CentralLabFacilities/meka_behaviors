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
from meka_flexbe_states.HandoverAdaptionExec import HandoverAdaptionExec
from meka_flexbe_behaviors.deterministicgazehandhandface_sm import DeterministicGazeHandHandFaceSM
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.check_condition_state import CheckConditionState
from meka_flexbe_behaviors.deterministicgazeiotpface_sm import DeterministicGazeiOTPFaceSM
from meka_flexbe_states.ForceMonitor import ForceMonitor
from flexbe_states.wait_state import WaitState
from meka_flexbe_states.GroupToPosture import GroupToPosture
from meka_flexbe_states.GazeAtReturn import GazeAtTargetReturn
from meka_flexbe_states.GazeAt import GazeAtTarget
from meka_flexbe_states.AdjustTorso import AdjustTorso
from meka_flexbe_states.InitHandTracking import InitHandTracking
from meka_flexbe_states.ToggleHandState import ToggleHandState
from meka_flexbe_states.RemoteRecord import RemoteRecord
from meka_flexbe_states.WaitForDoldButton import WaitForDoldButton
from meka_flexbe_states.RemoteRecordStop import RemoteRecordStop
from meka_flexbe_states.StopHandTracking import StopHandTracking
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
		self.add_parameter('num_runs', 10)

		# references to used behaviors
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

		# x:30 y:518, x:130 y:518
		_sm_adaption_0 = OperatableStateMachine(outcomes=['succeeded', 'error'], input_keys=['carrying'])

		with _sm_adaption_0:
			# x:37 y:138
			OperatableStateMachine.add('HandoverAdaptionExecPush',
										HandoverAdaptionExec(command=0, topic='/do_adaption', reality_damp=0.3, terminate_dist_override=0.04, terminate_timeout_override=1.0, fixed_orientation=True, terminate=True, use_reference_trajectory=False, joint_speed_limit=0.035),
										transitions={'succeeded': 'visualHOonReceive', 'error': 'error'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:226 y:290
			OperatableStateMachine.add('visualHOonReceive',
										CheckConditionState(predicate=lambda x: x),
										transitions={'true': 'HandoverAdaptionExecPush', 'false': 'succeeded'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'carrying'})


		# x:30 y:365, x:130 y:365
		_sm_graspgive_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['carrying'], output_keys=['carrying'])

		with _sm_graspgive_1:
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
		_sm_gazerobotneutral_2 = OperatableStateMachine(outcomes=['finished'])

		with _sm_gazerobotneutral_2:
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
		_sm_checkmoving_3 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_checkmoving_3:
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
		_sm_gazeuntiliotp_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('DeterministicGazeHandHandFace', 'done')]),
										('finished', [('checkMoving', 'finished')]),
										('failed', [('checkMoving', 'failed')])
										])

		with _sm_gazeuntiliotp_4:
			# x:52 y:144
			OperatableStateMachine.add('DeterministicGazeHandHandFace',
										self.use_behavior(DeterministicGazeHandHandFaceSM, 'AdaptionGazeAtHuman/Gaze/gazeUntiliOTP/DeterministicGazeHandHandFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'carrying_in': 'carrying'})

			# x:289 y:71
			OperatableStateMachine.add('checkMoving',
										_sm_checkmoving_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


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

			# x:511 y:170
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


		# x:30 y:365, x:130 y:365
		_sm_gaze_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['carrying'])

		with _sm_gaze_6:
			# x:167 y:75
			OperatableStateMachine.add('gazeUntiliOTP',
										_sm_gazeuntiliotp_4,
										transitions={'finished': 'DeterministicGazeiOTPFace', 'failed': 'DeterministicGazeiOTPFace'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:167 y:234
			OperatableStateMachine.add('DeterministicGazeiOTPFace',
										self.use_behavior(DeterministicGazeiOTPFaceSM, 'AdaptionGazeAtHuman/Gaze/DeterministicGazeiOTPFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:348 y:373, x:247 y:367, x:430 y:365, x:530 y:365
		_sm_pushinhandgazeathuman_7 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], conditions=[
										('finished', [('DeterministicGazeHandHandFace', 'done')]),
										('finished', [('forceDuringMovement', 'success')]),
										('finished', [('Adaption', 'succeeded')]),
										('failed', [('Adaption', 'error')])
										])

		with _sm_pushinhandgazeathuman_7:
			# x:48 y:143
			OperatableStateMachine.add('Adaption',
										_sm_adaption_0,
										transitions={'succeeded': 'finished', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Inherit, 'error': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:226 y:145
			OperatableStateMachine.add('forceDuringMovement',
										ForceMonitor(force_threshold=14.0, topic='/force_helper'),
										transitions={'success': 'finished'},
										autonomy={'success': Autonomy.Off})

			# x:412 y:138
			OperatableStateMachine.add('DeterministicGazeHandHandFace',
										self.use_behavior(DeterministicGazeHandHandFaceSM, 'PushInHandGazeatHuman/DeterministicGazeHandHandFace'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'carrying_in': 'carrying'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_togglehandgaze_8 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['carrying'], output_keys=['carrying'], conditions=[
										('failed', [('robotHand', 'target_not_found')]),
										('finished', [('GraspGive', 'finished')]),
										('failed', [('GraspGive', 'failed')])
										])

		with _sm_togglehandgaze_8:
			# x:46 y:133
			OperatableStateMachine.add('GraspGive',
										_sm_graspgive_1,
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


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_retreatgaze_10 = ConcurrencyContainer(outcomes=['finished'], conditions=[
										('finished', [('GazeRobotNeutral', 'finished')]),
										('finished', [('gohome', 'succeeded')]),
										('finished', [('gohome', 'error')])
										])

		with _sm_retreatgaze_10:
			# x:98 y:111
			OperatableStateMachine.add('gohome',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'succeeded': 'finished', 'error': 'finished'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:324 y:109
			OperatableStateMachine.add('GazeRobotNeutral',
										_sm_gazerobotneutral_2,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365, x:630 y:365, x:730 y:365, x:830 y:365, x:930 y:365
		_sm_adaptiongazeathuman_11 = ConcurrencyContainer(outcomes=['finished', 'failed', 'inMotionForce'], input_keys=['carrying'], conditions=[
										('finished', [('OpenLittleAfterWait', 'finished')]),
										('failed', [('OpenLittleAfterWait', 'failed')]),
										('finished', [('Gaze', 'finished')]),
										('failed', [('Gaze', 'failed')]),
										('inMotionForce', [('forceInMotion', 'success')]),
										('finished', [('handoverAdaption', 'succeeded')]),
										('failed', [('handoverAdaption', 'error')])
										])

		with _sm_adaptiongazeathuman_11:
			# x:61 y:160
			OperatableStateMachine.add('handoverAdaption',
										HandoverAdaptionExec(command='trigger', topic='/do_adaption', reality_damp=0.8, terminate_dist_override=0.08, terminate_timeout_override=10.0, fixed_orientation=True, terminate=True, use_reference_trajectory=True, joint_speed_limit=0.1),
										transitions={'succeeded': 'finished', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:615 y:158
			OperatableStateMachine.add('Gaze',
										_sm_gaze_6,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:256 y:162
			OperatableStateMachine.add('forceInMotion',
										ForceMonitor(force_threshold=24.0, topic='/force_helper'),
										transitions={'success': 'inMotionForce'},
										autonomy={'success': Autonomy.Off})

			# x:418 y:157
			OperatableStateMachine.add('OpenLittleAfterWait',
										_sm_openlittleafterwait_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})



		with _state_machine:
			# x:18 y:52
			OperatableStateMachine.add('resetAdaption',
										HandoverAdaptionReset(topic='/do_adaption'),
										transitions={'succeeded': 'start_button', 'error': 'resetAdaption'},
										autonomy={'succeeded': Autonomy.Off, 'error': Autonomy.Off})

			# x:631 y:256
			OperatableStateMachine.add('AdaptionGazeAtHuman',
										_sm_adaptiongazeathuman_11,
										transitions={'finished': 'PushInHandGazeatHuman', 'failed': 'AdjustTorso', 'inMotionForce': 'ToggleHandGaze'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'inMotionForce': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:206 y:369
			OperatableStateMachine.add('RetreatGaze',
										_sm_retreatgaze_10,
										transitions={'finished': 'stopRecording'},
										autonomy={'finished': Autonomy.Inherit})

			# x:534 y:50
			OperatableStateMachine.add('AdjustTorso',
										_sm_adjusttorso_9,
										transitions={'finished': 'InitHandTracking'},
										autonomy={'finished': Autonomy.Inherit})

			# x:691 y:65
			OperatableStateMachine.add('InitHandTracking',
										InitHandTracking(),
										transitions={'done': 'right_arm_hand_over_ready'},
										autonomy={'done': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:384 y:372
			OperatableStateMachine.add('ToggleHandGaze',
										_sm_togglehandgaze_8,
										transitions={'finished': 'RetreatGaze', 'failed': 'PushInHandGazeatHuman'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:668 y:139
			OperatableStateMachine.add('right_arm_hand_over_ready',
										GroupToPosture(hand='right', group='arm', posture='right_arm_hand_over_ready', posture_path='', non_blocking=True),
										transitions={'success': 'AdaptionGazeAtHuman', 'failure': 'AdaptionGazeAtHuman'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:624 y:364
			OperatableStateMachine.add('PushInHandGazeatHuman',
										_sm_pushinhandgazeathuman_7,
										transitions={'finished': 'ToggleHandGaze', 'failed': 'PushInHandGazeatHuman'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'carrying': 'carrying'})

			# x:355 y:41
			OperatableStateMachine.add('startRecording',
										RemoteRecord(topic='/meka/rosbagremote/record/named', pid=self.participant_id),
										transitions={'done': 'AdjustTorso'},
										autonomy={'done': Autonomy.Off},
										remapping={'carrying': 'carrying'})

			# x:228 y:43
			OperatableStateMachine.add('start_button',
										WaitForDoldButton(dold_button_topic='/dold_driver/state'),
										transitions={'done': 'startRecording'},
										autonomy={'done': Autonomy.Off})

			# x:24 y:363
			OperatableStateMachine.add('stopRecording',
										RemoteRecordStop(topic='/meka/rosbagremote/record/named'),
										transitions={'done': 'stopTracking'},
										autonomy={'done': Autonomy.Off})

			# x:22 y:295
			OperatableStateMachine.add('stopTracking',
										StopHandTracking(),
										transitions={'done': 'start_button'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
