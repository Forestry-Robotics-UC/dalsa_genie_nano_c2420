#!/usr/bin/env python
from __future__ import print_function
import cv2 as cv
import sys
import roslib
roslib.load_manifest('core_project')
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class NDVI_converter():
	def __init__(self):
		rospy.on_shutdown(self.shutdown)
		# Get ROS Params
		camera_topic = self.camera_chosen(rospy.get_param("Camera_chosen"))
		print ("Camera Topic: " + str(camera_topic))

		# Publishers
		self.ndvi_pub = rospy.Publisher("dalsa_camera/ndvi", Image, queue_size=100)
		self.set_extra_publishers()

		# CV Bridge
		self.bridge = CvBridge()

		# Subscribers
		self.dalsa_img = rospy.Subscriber(camera_topic, Image, self.camera_callback, 
			queue_size = 100)


	def set_extra_publishers(self):

		if rospy.get_param("Indexes/CVI") == True:
			self.cvi_pub = rospy.Publisher("cvi", Image, queue_size=100)
		if rospy.get_param("Indexes/Red") == True:
			self.red_pub = rospy.Publisher("red", Image, queue_size=100)
		if rospy.get_param("Indexes/Green") == True:
			self.green_pub = rospy.Publisher("green", Image, queue_size=100)
		if rospy.get_param("Indexes/NIR") == True:
			self.nir_pub = rospy.Publisher("nir", Image, queue_size=100)
		if rospy.get_param("Indexes/Normalized_Green") == True:
			self.ngreen_pub = rospy.Publisher("normalized_green", Image, queue_size=100)
		if rospy.get_param("Indexes/Normalized_NIR") == True:
			self.nnir_pub = rospy.Publisher("normalized_nir", Image, queue_size=100)
		if rospy.get_param("Indexes/Normalized_Red") == True:
			self.nred_pub = rospy.Publisher("normalized_red", Image, queue_size=100)
		if rospy.get_param("Indexes/TVI") == True:
			self.tvi_pub = rospy.Publisher("tvi", Image, queue_size=100)

	def camera_chosen(self, i):
		# Switcher for yaml file: choose which camera to subscribe.
		print ("Index Camera: "+ str(rospy.get_param("Camera_chosen")))
		self.switcher={
				0:rospy.get_param("Subscribers/Camera_720p/topic"),
				1:rospy.get_param("Subscribers/Camera_4k/topic"),
				2:rospy.get_param("Subscribers/Compressed_720p/topic"),
				3:rospy.get_param("Subscribers/Compressed_4k/topic")
		}

		return self.switcher.get(i, "Error: Invalid Option; No Camera chosen")

	def convert_image_cv(self, ros_image):
		# Convert ROS image to OpenCV
		try:
			cv_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
			return cv_image
		except CvBridgeError as e:
			print(e)

	def publish_image(self, image, publisher):
		# Simple definition to publish image in the publisher chosen
		# Convert OpenCV image to ROS image
		try:
			publisher.publish(self.bridge.cv2_to_imgmsg(image, encoding="passthrough"))
		except CvBridgeError as e:
				print (e)

	def extra_indexes (self, g_float, nir_float, r_float):
		# Publish all extra indexes if set as True on the camera.yaml file

		# Get Chlorophyll Vegetation Index
		if rospy.get_param("Indexes/CVI") == True:
			cvi = nir_float * ((r_float) / (g_float ** 2))
			# Transform float64 to uint8
			CVI = cv.normalize(src = cvi, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
			# Publish if Index is true at the yaml file
			self.publish_image(CVI, self.cvi_pub)

		# Get Normalized Green
		if rospy.get_param("Indexes/Normalized_Green") == True:
			n_green = (g_float / (nir_float + r_float + g_float))
			Normalized_Green = cv.normalize(src = n_green, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
			# Publish if Index is true at the yaml file
			self.publish_image(Normalized_Green, self.ngreen_pub)

		# Get Normalized NIR
		if rospy.get_param("Indexes/Normalized_NIR") == True:
			n_nir = ((nir_float) / (nir_float + r_float + g_float))
			Normalized_NIR = cv.normalize(src = n_nir, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
			# Publish if Index is true at the yaml file
			self.publish_image(Normalized_NIR, self.nnir_pub)

		# Get Normalized Red
		if rospy.get_param("Indexes/Normalized_Red") == True:
			n_red = (r_float / (nir_float + r_float + g_float))
			Normalized_Red = cv.normalize(src = n_red, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
			# Publish if Index is true at the yaml file
			self.publish_image(Normalized_Red, self.nred_pub)

		# Triangular Vegetation Index
		if rospy.get_param("Indexes/TVI") == True:
			tvi = 0.5 * (120*(nir_float - g_float) - 200*(r_float - g_float))
			TVI = cv.normalize(src = tvi, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
			# Publish if Index is true at the yaml file
			self.publish_image(TVI, self.tvi_pub)


	def camera_callback(self, data):

		cv_image = self.convert_image_cv(data)

		# Split Channels 
		(nir, r, g) = cv.split(cv_image)

		# Publish if Index is true at the yaml file
		if rospy.get_param("Indexes/NIR") == True:
			self.publish_image(nir, self.nir_pub)

		# Publish if Index is true at the yaml file
		if rospy.get_param("Indexes/Green") == True:
			self.publish_image(g, self.green_pub)

		# Publish if Index is true at the yaml file
		if rospy.get_param("Indexes/Red") == True:
			self.publish_image(r, self.red_pub)

		# Converting channels to float64 for index calculations
		r_float = r.astype(np.float)
		nir_float = nir.astype(np.float)
		g_float =  g.astype(np.float)
		# Ignoring division by 0
		np.seterr(divide='ignore', invalid='ignore')		
		# Get Normalized Difference Vegetation Index (NDVI)
		NDVI = (nir_float - r_float) / (nir_float + r_float)
		# Transform float64 to uint8
		ndvi_img = cv.normalize(src = NDVI, dst = None, alpha = 0, beta = 255, 
			norm_type=cv.NORM_MINMAX, dtype=cv.CV_16U)
		#Publish Image
		self.publish_image(ndvi_img, self.ndvi_pub)
		
		# Calculate and publish any extra indexes requested
		self.extra_indexes(g_float, nir_float, r_float)


	def shutdown(self):
		# Instructions on shutdown
		rospy.loginfo("Shutting Down")
		cv.destroyAllWindows()

def main():
	rospy.init_node('ndvi_multispectral', anonymous = True)
	ndvi = NDVI_converter()
	rospy.spin()

if __name__ == '__main__':
	try: 
		main()
	except rospy.ROSInterruptException:
		pass
	except KeyboardInterrupt:
		sys.exit(0)
