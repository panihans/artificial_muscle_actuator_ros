#!/usr/bin/env python3
import rospy
import actionlib
from am_soft_grip_msgs.msg import ActuatorFeedback, ChargeCommand, MeasureCommand, ResetCommand, ShortCommand, \
    StopCommand, GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult
from am_soft_grip_msgs.srv import GripperStatus, GripperStatusRequest, GripperStatusResponse

def move_gripper():
    client = actionlib.SimpleActionClient('/am_driver/gripper_command', GripperCommandAction)

    goal = GripperCommandGoal()
    goal.grip_direction = GripperCommandGoal.DIR_LEFT
    goal.grip_speed = GripperCommandGoal.MAX_SPEED

    rospy.wait_for_service('/am_driver/get_status')
    get_status = rospy.ServiceProxy('/am_driver/get_status', GripperStatus)
    while not rospy.is_shutdown():
        print(get_status())
        client.send_goal_and_wait(goal)
        client.wait_for_result()
        print(client.get_result())

        rospy.sleep(10)
        pass


def main():
    rospy.init_node("am_driver_client_example", log_level=rospy.DEBUG)
    rospy.logdebug('am soft grip driver client example starting')
    move_gripper()


if __name__ == '__main__':
    main()
