#!/usr/bin/env python

PACKAGE = 'amr_stage'
NODE = 'ranges_republisher'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from sensor_msgs.msg import Range
from amr_msgs.msg import Ranges

def rangesCallback(msg):
    if filter_ids:
        for i in filter_ids:
            try:
                ranges_pub.publish(msg.ranges[int(i)])
            except IndexError:
                continue
    else:
        for r in msg.ranges:
            ranges_pub.publish(r)

if __name__ == '__main__':
    rospy.init_node(NODE, anonymous=True)
    try:
        filter_ids = rospy.get_param('~filter_ids').split(',')
    except KeyError:
        filter_ids = False
    ranges_sub = rospy.Subscriber('ranges_in', Ranges, rangesCallback)
    ranges_pub = rospy.Publisher('ranges_out', Range)
    rospy.spin()
