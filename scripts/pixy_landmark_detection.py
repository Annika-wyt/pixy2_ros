#! /usr/bin/env python3
import rospy
import numpy as np
import tf2_ros

from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped, TransformStamped, Point, PointStamped, Quaternion, Vector3, Vector3Stamped, PoseArray, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Imu
import tf2_geometry_msgs
import tf.transformations

from visualization_msgs.msg import Marker as VM
from visualization_msgs.msg import MarkerArray as VMA

from pixy_camera.msg import ObjectInfoArrayStamped, ObjectBearing, ObjectBearingArrayStamped

class pixy_landmark_detection():
    def __init__(self):
        rospy.init_node('pixy_landmark_detection')
        self.NAMESPACE = rospy.get_namespace()
        self.DEBUG = rospy.get_param("~debug", False)

        self.getCameraInfo = False
        self.fx = None #camera_params_.K[0];
        self.fy = None #camera_params_.K[4];
        self.cx = None #camera_params_.K[2];
        self.cy = None #camera_params_.K[5];
        
        # Subscriber
        rospy.Subscriber(self.NAMESPACE + "pixy_node/camera_info", CameraInfo, self.cameraInfoCallback) 
        
        rospy.Subscriber(self.NAMESPACE + "pixy_node/objectsInfo", ObjectInfoArrayStamped, self.objInfoCallback)

        # Publisher
        self.ObjBearingPub = rospy.Publisher('~objectsBearing', ObjectBearingArrayStamped, queue_size=10)

    def objInfoCallback(self, msg):
        msgArray = ObjectBearingArrayStamped()
        msgArray.header = msg.header
        if self.getCameraInfo:
            for object in msg.objects:
                msgObject = ObjectBearing()
                bearing = [(self.cx - object.center_x)/self.fx, (self.cy - object.center_y)/self.fy, -1]
                bearing /= np.linalg.norm(bearing)
                msgObject.id = object.signature
                msgObject.bearing_x = bearing[0]
                msgObject.bearing_y = bearing[1]
                msgObject.bearing_z = bearing[2]
                msgArray.bearing.append(msgObject)
        self.ObjBearingPub.publish(msgArray)

    def cameraInfoCallback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.getCameraInfo = True
        #TODO: maybe unregister()??

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    pixy_landmark_detection().run()
