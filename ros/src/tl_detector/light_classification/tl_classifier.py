import numpy as np
import tensorflow as tf

import rospy
from styx_msgs.msg import TrafficLight
import label_map_util
import visualization_utils as vis_util
import os
import cv2
from functools import partial

CLASSIFICATION_THRESHOLD = 0.5  


class TLClassifier(object):
	def __init__(self):
		self.modelpath = rospy.get_param('~model_path')
		# PATH_TO_MODEL = 'models/trial19_ssd_inception_sim_frozen_inference_graph.pb' # model resnet-udacity-sim-large-10-regions
		PATH_TO_MODEL = self.modelpath

		self.saved_image_limit = 500
		self.saved_image_counter = 1
		self.save_images = False

		self.readsize = 1024

		# Build the model
		self.detection_graph = tf.Graph()

		# Create the graph
		with self.detection_graph.as_default():
			graph_def = tf.GraphDef()
			with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
				serialized_graph = fid.read()
				graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(graph_def, name='')

			self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
			self.path = './light_classification/Conversion_label_map.pbtxt'

		# Create a reusable sesion attribute
		self.sess = tf.Session(graph=self.detection_graph)

		PATH_TO_LABELS = self.path
		NUM_CLASSES = 4

		label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
		categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
			use_display_name=True)
		self.category_index = label_map_util.create_category_index(categories)
		self.count = 1

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""

		with self.detection_graph.as_default():
			image_np_expanded = np.expand_dims(image, axis=0)
			(boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores,
				self.detection_classes, self.num_detections],
				feed_dict={self.image_tensor: image_np_expanded})

	
		#rospy.loginfo('scores: {}'.format(scores))
		#rospy.loginfo('Classe: {}'.format(classes))

		# create np arrays
		boxes = np.squeeze(boxes)
		scores = np.squeeze(scores)
		classes = np.squeeze(classes).astype(np.int32)

		if self.save_images == True and self.saved_image_counter <= self.saved_image_limit:
			vis_util.visualize_boxes_and_labels_on_image_array(
				image, boxes, classes, scores,
				self.category_index,
				use_normalized_coordinates=True,
				line_thickness=4)
			if not (os.path.exists("./tl_images_inference_test")):
				os.mkdir("./tl_images_inference_test")
			cv2.imwrite("./tl_images_inference_test/inference_image{0:0>4}.jpeg".format(self.saved_image_counter),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
			rospy.loginfo('Image : inference_image{0:0>4}.jpeg'.format(self.saved_image_counter))
			self.saved_image_counter += 1

		#rospy.loginfo('Score: {}'.format(scores[0]))
		max_score = 0
		ind_score = 0
		flag_score = 0
		for i in range(len(scores)):
			if scores[i] > CLASSIFICATION_THRESHOLD:
				if scores[i] > max_score:
					max_score = scores[i]
					ind_score = i
					flag_score = 1
		if flag_score == 1: 
			#rospy.loginfo('Classe: {}'.format(classes[0]))
			if classes[ind_score] == 1:
				rospy.loginfo('GREEN')
				return TrafficLight.GREEN
			elif classes[ind_score] == 2:
				rospy.loginfo('RED')
				return TrafficLight.RED
			elif classes[ind_score] == 3 or classes[ind_score] == 7:
				rospy.loginfo('YELLOW')
				return TrafficLight.YELLOW
			else:
				rospy.loginfo('UNKNOWN')
				return TrafficLight.UNKNOWN
		else:
			rospy.loginfo('UNKNOWN')
			return TrafficLight.UNKNOWN
		

