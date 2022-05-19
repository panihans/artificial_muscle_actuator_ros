#!/usr/bin/env python3
import rospy
import actionlib
from am_soft_grip_msgs.msg import ActuatorFeedback, ChargeCommand, MeasureCommand, ResetCommand, ShortCommand, \
    StopCommand, GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult
from am_soft_grip_msgs.srv import GripperStatus


class AMSoftGripDriver:
    def __init__(self):
        self.charge_pub = rospy.Publisher('/am_act/charge', ChargeCommand)
        self.measure_pub = rospy.Publisher('/am_act/measure', MeasureCommand)
        self.reset_pub = rospy.Publisher('/am_act/reset', ResetCommand)
        self.short_pub = rospy.Publisher('/am_act/short', ShortCommand)
        self.stop_pub = rospy.Publisher('/am_act/stop', StopCommand)
        self.status_sub = rospy.Subscriber('/am_act/status', ActuatorFeedback, self.feedback_cb)
        self.gripper_status_srv = rospy.Service('/am_driver/get_status', GripperStatus, self.get_status_cb)
        self.gripper_action_server = actionlib.SimpleActionServer('/am_driver/gripper_command', GripperCommandAction,
                                                                  self.gripper_execute_cb, auto_start=False)
        self.gripper_action_server.start()
        self.feedback = ActuatorFeedback()
        self.feedback.actuator_state = -1

    def feedback_cb(self, feedback: ActuatorFeedback):
        self.feedback = feedback

    def get_status_cb(self, request: GripperStatus):
        pass

    def is_actuator_stopped(self):
        return self.feedback.actuator_state == ActuatorFeedback.OPEN_CIRCUIT

    def is_actuator_charging(self):
        return self.feedback.actuator_state == ActuatorFeedback.CHARGING

    def is_actuator_shorting(self):
        return self.feedback.actuator_state == ActuatorFeedback.SHORT_CIRCUIT

    def is_actuator_measuring(self):
        return self.feedback.actuator_state == ActuatorFeedback.MEASURING

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
        cmd.sent = not rospy.Time.now()
        self.short_pub.publish(cmd)
        return cmd.sent

    def gripper_execute_cb(self, goal: GripperCommandGoal):
        grip_starting = self.feedback.icl_coulomb_counter
        action_feedback = GripperCommandFeedback()
        action_result = GripperCommandResult()
        charge_start_time = None
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            action_feedback.relative_grip_change = grip_starting - self.feedback.icl_coulomb_counter
            # preempting
            if self.gripper_action_server.is_preempt_requested():
                if not self.is_actuator_stopped():
                    self.send_stop()
                else:
                    action_result.succeeded = False
                    action_result.relative_grip_change = action_feedback.relative_grip_change
                    return self.gripper_action_server.set_preempted(action_result)
            # start charge
            if not self.is_latest_command(charge_start_time):
                v = 0
                if goal.grip_direction == goal.DIR_LEFT:
                    v = 1.3
                elif goal.grip_direction == goal.DIR_RIGHT:
                    v = -1.3
                else:
                    v = 0
                self.send_charge(v, 0.001)
            elif self.is_actuator_charging():  # charge progress
                self.gripper_action_server.publish_feedback(action_feedback)
            elif self.is_actuator_stopped():  # charge done
                action_result.succeeded = True
                action_result.relative_grip_change = action_feedback.relative_grip_change
                return self.gripper_action_server.set_succeeded(action_result)
            else:  # actuator in wrong state
                action_result.succeeded = False
                action_result.relative_grip_change = action_feedback.relative_grip_change
                return self.gripper_action_server.set_aborted()

            rospy.logdebug('goal=' + str(goal.grip_direction) + ' preempt=' + str(
                self.gripper_action_server.is_preempt_requested()))
            r.sleep()

        pass


def main():
    rospy.init_node("am_soft_grip_driver", log_level=rospy.DEBUG)
    rospy.logdebug('am soft grip driver starting')
    driver = AMSoftGripDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
