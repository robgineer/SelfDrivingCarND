from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import os
import cv2
import rospy

TRAFFIC_LIGHT_LABEL = 10


class TLClassifier(object):
    def __init__(self, model_path = None):
        # TODO load classifier

        # loading SSD for the localization of traffic lights
        # the lines below have been modified based on the tutotial in https://github.com/udacity/CarND-Object-Detection-Lab

        if model_path is None:
            #rp = rospkg.RosPack()
            # model folder in tl_detector folder
            model_path = os.path.join(os.path.dirname(os.getcwd()), 'model')

        detection_model_path = os.path.join(model_path, 'model_detection.pb')
        classification_model_path = os.path.join(model_path, 'model_classification.pb')
        # check if models exists and output error?

        self.config = tf.ConfigProto()  # protocol msgs
        # load graphs
        self.detection_graph = self.load_graph(detection_model_path)
        self.classification_graph = self.load_graph(classification_model_path)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        self.config = tf.ConfigProto()
        # Create TF sessions
        self.sess_detection = tf.Session(graph=self.detection_graph, config=self.config)
        self.sess_classification = tf.Session(graph=self.classification_graph, config=self.config)
        self.in_graph = self.classification_graph.get_tensor_by_name('input_1:0')
        self.out_graph = self.classification_graph.get_tensor_by_name('output_0:0')

        rospy.loginfo("loading done...")
        # Create Output from classification
        # self.index2msg = {0: TrafficLight.GREEN, 1: TrafficLight.RED, 2: TrafficLight.YELLOW}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        state = None
        cropped_img_lst = self.tl_detection(image)
        if cropped_img_lst is None:
            return TrafficLight.UNKNOWN
        else:
            #iterate trough all found traffic lights
            for cropped_image in cropped_img_lst:
                #classsify state
                traffic_light = cv2.resize(cropped_image, (32, 32))
                state = self.classify_it(traffic_light)
                rospy.loginfo("traffic light color: {}".format(state))
                # traffic light is RED
                if state == 0:
                    # even if only one traffic light is classified as red, return RED
                    return TrafficLight.RED
                # yellow case (less severe)
                elif state == 2:
                    return TrafficLight.YELLOW
                elif state == 1:
                    state = TrafficLight.GREEN
        # returns green only if RED and YELLOW have not been detected in any cropped image
        return state

    def tl_detection(self, image):
        h_, w_, _ = image.shape
       # print("checkpoint img shape: {}".format(image.shape))
        image_np = np.expand_dims(image, axis=0)
        # with tf.Session(graph=self.detection_graph) as sess:
        with self.sess_detection.as_default(), self.detection_graph.as_default():
            # Actual detection.
            # (boxes, scores, classes) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes],
            #                                    feed_dict={self.image_tensor: image_np})

            boxes, scores, classes = self.sess_detection.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_np})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            confidence_cutoff = 0.5
            # Filter boxes with a confidence score less than `confidence_cutoff`
            boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
            # The current box coordinates are normalized to a range between 0 and 1.
            # This converts the coordinates actual location on the image.
            box_coords = self.to_image_coords(boxes, h_, w_)
            # Each class with be represented by a differently colored box
            # draw_boxes(image, box_coords, classes)
            print("len classes: {}, len boxes: {}".format(len(classes), len(boxes)))
            return self.extract(image, box_coords, classes)

    def classify_it(self, image):
        with self.sess_classification.as_default(), self.classification_graph.as_default():
            sfmax = list(
                self.sess_classification.run(tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [image]}))))
            sf_ind = sfmax.index(max(sfmax))
        return sf_ind

    def close(self):
        self.sess.close()

    # helper functions modified after https://github.com/udacity/CarND-Object-Detection-Lab

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords

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

    def extract(self, img, boxes, classes):
        listofimages = list()
        # iterate through all found boxes in the image
        for i in range(len(boxes)):
            # get coordinates
            y1, x1, y2, x2 = boxes[i, ...]
            class_id = int(classes[i])
            # crop image and store into list
            if class_id == TRAFFIC_LIGHT_LABEL:
                img2 = img.copy()
                img2 = img2[int(y1):int(x1), int(y2):int(x2)]
                listofimages.append(img2)
        return listofimages
