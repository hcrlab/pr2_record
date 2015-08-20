#!/usr/bin/env python

from pr2_record.srv import *
from sensor_msgs.msg import PointCloud2, Image
from tf2_msgs.msg import TFMessage
import rospy
import rosbag

class BagServer:
    """Manages open bags and records to them."""
    def __init__(self):
        self.pointcloud_topic = "/head_mount_kinect/depth_registered/points"
        self.image_topic = "/head_mount_kinect/rgb/image_rect_color"
        self.pointcloud_subscriber = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.image_callback)
 
    def pointcloud_callback(self, msg):
        self.pointcloud_msg = msg
      
    def image_callback(self, msg):
        self.image_msg = msg

    def tf_callback(self, msg):
        pass
        # self.bag.write("tf", msg)

    def open_bag_file(self, req):
        print("Attempting to open bag file: " + req.filename)
        success = False
        try:
            self.bag = rosbag.Bag(req.filename, 'w')
            self.filename = req.filename
            print("Successfully opened bag file: " + req.filename)
            self.tf_subscriber = rospy.Subscriber("tf", TFMessage, self.tf_callback)
            success = True
        except:
            print("Could not open bag file: " + req.filename)
        return OpenBagFileResponse(success)
    
    def close_bag_file(self, req):
        print("Attempting to close bag file: " + self.filename)
        success = False
        try:
            self.bag.close()
            print("Successfully closed bag file: " + self.filename)
            self.tf_subscriber.unregister()
            success = True
        except:
            print("Could not close bag file: " + self.filename)
        return CloseBagFileResponse(success)
    
    def record_depth_frame(self, req):
        if hasattr(self, 'filename'):
            print("Attempting to record depth frame to:" + self.filename)
        else:
            print("RecordDepthFrame failed ... it looks like you didn't call OpenBagFile")
            return RecordDepthFrameResponse(False)
        success = False
        try:
            self.bag.write(self.pointcloud_topic, self.pointcloud_msg)
            self.bag.write(self.image_topic, self.image_msg)
            success = True
        except:
            print("Could not write to bag. Have you called OpenBagFile?")
        return RecordDepthFrameResponse(success)

def pr2_record_server():
    bag_server = BagServer()
    rospy.init_node('pr2_record_server')
    s1 = rospy.Service('open_bag_file', OpenBagFile, bag_server.open_bag_file)
    s2 = rospy.Service('close_bag_file', CloseBagFile, bag_server.close_bag_file)
    s3 = rospy.Service('record_depth_frame', RecordDepthFrame, bag_server.record_depth_frame)
    print "Ready to record bag files."
    rospy.spin()

if __name__ == "__main__":
    pr2_record_server()
