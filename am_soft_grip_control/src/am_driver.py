import rospy
import actionlib
from am_actuator_msgs.msg import ActuatorFeedback, ChargeCommand, MeasureCommand, ResetCommand, ShortCommand, \
    StopCommand


def main():
    rospy.init_node("am_soft_grip_driver", log_level=rospy.DEBUG)


if __name__ == '__main__':
    main()
