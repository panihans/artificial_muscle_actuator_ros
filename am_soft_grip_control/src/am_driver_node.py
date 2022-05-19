#!/usr/bin/env python3
import rospy
from am_driver import AMSoftGripDriver


def main():
    rospy.init_node("am_soft_grip_driver", log_level=rospy.DEBUG)
    rospy.logdebug('am soft grip driver starting')
    driver = AMSoftGripDriver()
    rospy.spin()


if __name__ == '__main__':
    main()
