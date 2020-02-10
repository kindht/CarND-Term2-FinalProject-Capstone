from styx_msgs.msg import TrafficLight

import rospy
import yaml

import tensorflow as tf
import numpy as np
import cv2
import sys
import os

CUT_OFF = 0.7  # scores threshold 
GRAPH_FILE_SIM  = '/light_classification/models/sim/frozen_inference_graph.pb'   # simulator
GRAPH_FILE_SITE = '/light_classification/models/site/frozen_inference_graph.pb'  # real site

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        sys.path.append("..")

        config_string = rospy.get_param("/traffic_light_config")  # launch file will distinguish either site or sim
        config = yaml.load(config_string)  # config is a  dictionary
        
        self.is_site = config['is_site']  # site or simulator 

        if (self.is_site):
            self.graph_path = os.getcwd() + GRAPH_FILE_SITE
        else:
            self.graph_path = os.getcwd() + GRAPH_FILE_SIM
        # os.getcwd()=/home/student/CarND-Capstone/ros/src/tl_detector
        rospy.logwarn('TLClassifier self.is_site={}, graph_path={}'.format(self.is_site, self.graph_path))

        detection_graph = self.load_graph(self.graph_path) 

        self.sess = tf.Session(graph=detection_graph)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = self.sess.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.sess.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.sess.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.sess.graph.get_tensor_by_name('detection_classes:0')
        
        self.light_state = TrafficLight.RED # initialize as RED for safety at the begining

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        #TODO implement light color prediction
        #draw_img = Image.fromarray(img)
        boxes, scores, classes = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                                feed_dict={self.image_tensor: np.expand_dims(image, 0)})
        # Remove unnecessary dimensions
        # boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        #rospy.logwarn('TLClassifier scores={}'.format(scores))        
        # Filter with a confidence score 
        confidence_cutoff = CUT_OFF
        
        #classes must get alighed with what is defined in label_map.pbtxt
        if (scores[0] > CUT_OFF):
            rospy.logwarn("scores[0]={} > CUT_OFF={}".format(scores[0], CUT_OFF))
            if (classes[0] == 1.0):  
                self.light_state = TrafficLight.GREEN
            elif  (classes[0] == 2.0):
                self.light_state = TrafficLight.RED
            elif  (classes[0] == 3.0):
                self.light_state = TrafficLight.YELLOW

        #rospy.logwarn("tl_classifier: get_classification return self.light_state={}".format(self.light_state))
        return self.light_state
   

    def filter_scores(self, min_score, scores, classes):
        """Return boxes with a confidence >= min_score"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
 
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_scores, filtered_classes

    
    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph
