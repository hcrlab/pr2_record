#!/usr/bin/env python
import rospy
from pr2_record.srv import *

rospy.init_node('pr2_record_test')

open_bag_service = rospy.ServiceProxy('open_bag_file', OpenBagFile)
record_depth_service = rospy.ServiceProxy('record_depth_frame', RecordDepthFrame)
close_bag_service = rospy.ServiceProxy('close_bag_file', CloseBagFile)

open_bag_service('/home/djbutler/test1.bag')
rospy.sleep(1)
close_bag_service()

open_bag_service('/home/djbutler/test2.bag')
rospy.sleep(1)
record_depth_service()
rospy.sleep(1)
close_bag_service()

open_bag_service('/home/djbutler/test3.bag')
rospy.sleep(1)
record_depth_service()
rospy.sleep(2)
record_depth_service()
rospy.sleep(2)
close_bag_service()

