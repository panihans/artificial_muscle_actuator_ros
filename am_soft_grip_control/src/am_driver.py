import rospy
import actionlib
from am_soft_grip_msgs.msg import ActuatorFeedback, ChargeCommand, MeasureCommand, ResetCommand, ShortCommand, \
    StopCommand, GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult
from am_soft_grip_msgs.srv import GripperStatus, GripperStatusRequest, GripperStatusResponse

from ema import EMA
from math import nan
from threading import Semaphore


class AMSoftGripDriver:
    def __init__(self):
        self.feedback = ActuatorFeedback()
        self.feedback.actuator_state = -1
        self.icl_charging_avg = EMA(0.25)
        self.sema = Semaphore()

        self.charge_pub = rospy.Publisher('/am_act/charge', ChargeCommand)
        self.measure_pub = rospy.Publisher('/am_act/measure', MeasureCommand)
        self.reset_pub = rospy.Publisher('/am_act/reset', ResetCommand)
        self.short_pub = rospy.Publisher('/am_act/short', ShortCommand)
        self.stop_pub = rospy.Publisher('/am_act/stop', StopCommand)
        self.feedback_sub = rospy.Subscriber('/am_act/feedback', ActuatorFeedback, self.feedback_cb)
        self.gripper_status_srv = rospy.Service('/am_driver/get_status', GripperStatus, self.get_status_cb)
        self.gripper_action_server = actionlib.SimpleActionServer('/am_driver/gripper_command', GripperCommandAction,
                                                                  self.gripper_execute_cb, auto_start=False)
        self.gripper_action_server.start()

    def feedback_cb(self, feedback: ActuatorFeedback):
        self.feedback = feedback
        self.icl_charging_avg.average(feedback.icl_charging_voltage)

    def get_status_cb(self, request: GripperStatusRequest):
        resp = GripperStatusResponse()
        available = False
        if self.gripper_action_server.is_active():
            available = self.feedback.actuator_state != self.feedback.SHORT_CIRCUIT
        else:
            r = rospy.Rate(5)
            start = rospy.Time.now()
            while not self.sema.acquire(blocking=False):
                rospy.logdebug('service blocked by action')
                if rospy.Time.now().to_sec() - start.to_sec() > 2:
                    rospy.logdebug('service timeout1')
                    break
                r.sleep()
            measure_start_time = None
            while not rospy.is_shutdown():
                # rospy.logdebug(rospy.Time.now().to_sec() - start.to_sec())
                rospy.logdebug(self.feedback)
                if rospy.Time.now().to_sec() - start.to_sec() > 2:
                    rospy.logdebug('service timeout2')
                    self.send_stop()
                    break
                elif not self.is_latest_command(measure_start_time):
                    rospy.logdebug('service send_measure')
                    measure_start_time = self.send_measure()
                elif self.is_actuator_measuring():
                    rospy.logdebug('service is_measuring')
                    measure_start_time = self.send_stop()
                elif self.is_actuator_stopped():
                    rospy.logdebug('service send_stop')
                    available = True
                    break
                r.sleep()
        self.sema.release()
        if available:
            resp.grip_potential_left = max(0, min(50 - 41.6667 * self.feedback.icl_open_circuit_voltage, 100))
            resp.grip_potential_right = max(0, min(41.6667 * self.feedback.icl_open_circuit_voltage + 50, 100))
            resp.error = False
        else:
            resp.error = True
            resp.grip_potential_left = nan
            resp.grip_potential_right = nan
        return resp

    def is_actuator_charging(self):
        return self.feedback.actuator_state == ActuatorFeedback.CHARGING

    def is_actuator_measuring(self):
        return self.feedback.actuator_state == ActuatorFeedback.MEASURING

    def is_actuator_shorting(self):
        return self.feedback.actuator_state == ActuatorFeedback.SHORT_CIRCUIT

    def is_actuator_stopped(self):
        return self.feedback.actuator_state == ActuatorFeedback.OPEN_CIRCUIT

    def is_latest_command(self, t: [rospy.Time, None]):
        return self.feedback.last_command == t

    def send_charge(self, v, a):
        cmd = ChargeCommand()
        cmd.icl_target_voltage = v
        cmd.icl_current_limit = a
        cmd.sent = rospy.Time.now()
        self.charge_pub.publish(cmd)
        return cmd.sent

    def send_stop(self):
        cmd = StopCommand()
        cmd.sent = rospy.Time.now()
        self.stop_pub.publish(cmd)
        return cmd.sent

    def send_short(self):
        cmd = ShortCommand()
        cmd.sent = rospy.Time.now()
        self.short_pub.publish(cmd)
        return cmd.sent

    def send_measure(self):
        cmd = MeasureCommand()
        cmd.sent = rospy.Time.now()
        self.measure_pub.publish(cmd)
        return cmd.sent

    def send_reset(self):
        cmd = ResetCommand()
        cmd.sent = rospy.Time.now()
        self.reset_pub.publish(cmd)
        return cmd.sent

    def gripper_execute_cb(self, goal: GripperCommandGoal):
        grip_starting = self.feedback.icl_coulomb_counter
        action_feedback = GripperCommandFeedback()
        action_result = GripperCommandResult()
        charge_start_time = None
        charge_short_time = None
        r = rospy.Rate(1)
        while not self.sema.acquire(blocking=False):
            rospy.logdebug('action blocked by service')
            r.sleep()
        while not rospy.is_shutdown():
            action_feedback.relative_grip_change = self.feedback.icl_coulomb_counter - grip_starting
            # preempting
            if self.gripper_action_server.is_preempt_requested():
                if not self.is_actuator_stopped():
                    self.send_stop()
                else:
                    action_result.succeeded = False
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    self.gripper_action_server.set_preempted(action_result)
                    break
            if goal.grip_direction == goal.DIR_RESET:
                if not self.is_latest_command(charge_short_time):
                    charge_short_time = self.send_measure()
                elif self.is_actuator_measuring():
                    charge_short_time = self.send_short()
                elif self.is_actuator_shorting():
                    if -0.01 < self.icl_charging_avg.avg < 0.01:
                        self.send_reset()
                        charge_short_time = self.send_stop()
                elif self.is_actuator_stopped():
                    action_result.succeeded = True
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    self.gripper_action_server.set_succeeded(action_result)
                    break
                else:  # actuator in wrong state
                    action_result.succeeded = False
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    self.gripper_action_server.set_aborted()
                    break
            else:
                # start charge
                if not self.is_latest_command(charge_start_time):
                    rospy.logdebug('setting_charge()' + str(charge_start_time))
                    v = 0
                    if goal.grip_direction == goal.DIR_LEFT:
                        v = 1.3
                    else:
                        v = -1.3
                    speed = 0.001 * max(goal.min_speed, min(goal.grip_speed, goal.max_speed))
                    charge_start_time = self.send_charge(v, speed)
                elif self.is_actuator_charging():  # charge progress
                    rospy.logdebug('charging()')
                    self.gripper_action_server.publish_feedback(action_feedback)
                elif self.is_actuator_stopped():  # charge done
                    charge_start_time = self.send_measure()
                elif self.is_actuator_measuring():
                    # rospy.sleep(10)
                    # rospy.logdebug(self.feedback.icl_open_circuit_voltage)
                    # if self.feedback.icl_open_circuit_voltage < 1.2 or self.feedback.icl_open_circuit_voltage > -1.2:
                    #     # restart charge condition
                    #     rospy.logdebug('restart charge')
                    #     charge_start_time = None
                    # else:
                    rospy.logdebug('stopped()')
                    action_result.succeeded = True
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    self.gripper_action_server.set_succeeded(action_result)
                    break
                else:  # actuator in wrong state
                    rospy.logdebug('else()')
                    action_result.succeeded = False
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    self.gripper_action_server.set_aborted()
                    break

            rospy.logdebug('goal=' + str(goal.grip_direction) +
                           ' preempt=' + str(self.gripper_action_server.is_preempt_requested()) +
                           ' state=' + str(self.feedback.actuator_state) +
                           ' ocv=' + str(self.feedback.icl_open_circuit_voltage) +
                           ' ccv=' + str(self.feedback.icl_charging_voltage) +
                           ' col=' + str(self.feedback.icl_coulomb_counter) +
                           ' lt=' + str(self.feedback.last_command)
                           )
            r.sleep()
        self.sema.release()
