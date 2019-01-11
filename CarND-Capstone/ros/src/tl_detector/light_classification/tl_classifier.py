from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os
import rospy
import pathlib

DETECT_THRES = 0.5

class TLClassifier(object):
    def __init__(self, is_site):
        #TODO load classifier
        if is_site: # Use the classfier for site camera
            PATH_TO_MODEL = str(pathlib.Path(__file__).parent) + '/fast_rcnn_incept_3000/frozen_inference_graph.pb'
        else: # Use the classfier for simulator camera
            PATH_TO_MODEL = str(pathlib.Path(__file__).parent) + '/simulator_fine_tune_2000/frozen_inference_graph.pb'

        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        
        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)  
            (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})

        res_label = classes[0][0]
        res_score = scores[0][0]

        if res_score > DETECT_THRES:
            if res_label == 1:
                result = TrafficLight.RED
                rospy.logwarn("RED light detected with score: {0}".format(res_score))
            elif res_label == 2:
                result = TrafficLight.YELLOW
                rospy.logwarn("YELLOW light detected with score: {0}".format(res_score))
            elif res_label == 3:
                result = TrafficLight.GREEN
                rospy.logwarn("GREEN light detected with score: {0}".format(res_score))
            else:
                result = TrafficLight.UNKNOWN
                rospy.logwarn("NO light detected with score: {0}".format(res_score))
        else:
            result = TrafficLight.UNKNOWN
            rospy.logwarn("NO significant result with score: {0}".format(res_score))

        return result
