#!/usr/bin/env python2

"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
import roslib; roslib.load_manifest('dalsa_genie_nano_c2420')

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse dalsa camera 4k
    dalsa_camera_info_msg = CameraInfo()
    dalsa_camera_info_msg.header.frame_id = 'dalsa_optical_frame'
    dalsa_camera_info_msg.width = calib_data["dalsa"]["image_width"]
    dalsa_camera_info_msg.height = calib_data["dalsa"]["image_height"]
    dalsa_camera_info_msg.K = calib_data["dalsa"]["camera_matrix"]["data"]
    dalsa_camera_info_msg.D = calib_data["dalsa"]["distortion_coefficients"]["data"]
    dalsa_camera_info_msg.R = calib_data["dalsa"]["rectification_matrix"]["data"]
    dalsa_camera_info_msg.P = calib_data["dalsa"]["projection_matrix"]["data"]
    dalsa_camera_info_msg.distortion_model = calib_data["dalsa_720p"]["distortion_model"]


    # Parse dalsa camera 720p
    dalsa_720p_camera_info_msg = CameraInfo()
    dalsa_camera_info_msg.header.stamp = rospy.Time.now()
    dalsa_720p_camera_info_msg.header.frame_id = "dalsa_optical_frame"
    dalsa_720p_camera_info_msg.width = calib_data["dalsa_720p"]["image_width"]
    dalsa_720p_camera_info_msg.height = calib_data["dalsa_720p"]["image_height"]
    dalsa_720p_camera_info_msg.K = calib_data["dalsa_720p"]["camera_matrix"]["data"]
    dalsa_720p_camera_info_msg.D = calib_data["dalsa_720p"]["distortion_coefficients"]["data"]
    dalsa_720p_camera_info_msg.R = calib_data["dalsa_720p"]["rectification_matrix"]["data"]
    dalsa_720p_camera_info_msg.P = calib_data["dalsa_720p"]["projection_matrix"]["data"]
    dalsa_720p_camera_info_msg.distortion_model = calib_data["dalsa_720p"]["distortion_model"]
    return dalsa_camera_info_msg, dalsa_720p_camera_info_msg

if __name__ == "__main__":
    rospy.init_node("dalsa_camera_info", anonymous=False)

    filename = rospy.get_param("/dalsa_camera_info/camera_info_path") 

    # Parse yaml file
    dalsa_camera_info_msg, dalsa_720p_camera_info_msg  = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    publisher_dalsa = rospy.Publisher("dalsa_camera/camera_info", CameraInfo,
                                         queue_size=10, latch=True)
    publisher_dalsa_720p = rospy.Publisher("dalsa_camera_720p/camera_info", 
                                        CameraInfo, queue_size=10, latch=True)

    # rospy.spin()
    rate = rospy.Rate(50)

    # Run publisher
    while not rospy.is_shutdown():
        dalsa_camera_info_msg.header.stamp = rospy.Time.now()
        dalsa_720p_camera_info_msg.header.stamp = rospy.Time.now()

        publisher_dalsa.publish(dalsa_camera_info_msg)
        publisher_dalsa_720p.publish(dalsa_720p_camera_info_msg)
        rate.sleep()

