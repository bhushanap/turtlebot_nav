
# Final, AE 7785
# Richard Agbeyibor & Bhushan Pawaskar
# Train KNN, classify sign

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
import csv
import time
import numpy as np

import cv2
from cv_bridge import CvBridge

class ReadSign(Node):
    
    def __init__(self):
        # Calls the superclass constructor from Node
        super().__init__('readSign')

        # Creates a KNN instantiation
        self.knn = cv2.ml.KNearest_create()
        self.imageDirectory =  '/home/burger/2022Fimgs/'
        self.labels_filename = 'labels.txt'
        self.height = int(48/4)
        self.width = int(64/4)
        # Train the KNN
        self.trainKNN(self.imageDirectory, self.labels_filename)

        # Subscriber to the /camera/image_raw topic
        qos_profile = QoSProfile(depth=5)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST 
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT 
        self._img_subscriber = self.create_subscription(CompressedImage, 
                                                         '/camera/image/compressed',
                                                         self._image_callback,
                                                         qos_profile)
        self._img_subscriber # supposedly prevents unused variable warning

        # Publisher for processed CompressedImage msg to the /sign/image/compressed topic
        self._img_publisher = self.create_publisher(CompressedImage, 
                                                    '/sign/image/compressed',
                                                    1)
        self._img_publisher # to prevent unused variable warning
        self.processed_frame = CompressedImage()


        # Publisher for sign classification
        self._sign_publisher = self.create_publisher(String, 
                                                    '/sign/label',
                                                    1)
        self._sign_publisher # supposedly prevents unused variable warning
        self.sign = String()

        # Create a timer for publishing sign classifications
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)



    def trainKNN(self, imageDirectory, labels):
        
        with open(imageDirectory + labels, 'r') as f:
            reader = csv.reader(f)
            lines = list(reader)
        train = np.zeros((len(lines), self.height, self.width))
        train_labels = np.zeros((len(lines)), dtype=int)

        for i in range(0, len(lines)):
            # Load original image
            original_img = cv2.imread(imageDirectory+lines[i][0]+".png",1)

            # HACK Initialize self_imgBGR to avoid future null array errors
            self._imgBGR = original_img.copy()

            # Convert to HSV and split
            hsv = cv2.cvtColor(original_img, cv2.COLOR_BGR2HSV)
            h,s,v = cv2.split(hsv)
            # Take a binary threshold
            ret,thresh = cv2.threshold(s,120,255,cv2.THRESH_BINARY)
            proc_img = thresh

            # Find contours if any
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Crop by largest contour
            if len(contours) != 0:
                #draw all contours in gray
                cont_img = thresh
                cv2.drawContours(cont_img, contours, -1, (100,100,100), 1)
                #find and draw a bounding rectangle around the biggest contour
                c = max(contours, key = cv2.contourArea)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(cont_img,(x,y),(x+w,y+h),(255,255,255),2)
                cropped_img = thresh[y:y+h, x:x+w]
                proc_img = cropped_img
                

            resized_img = cv2.resize(proc_img, (self.width, self.height), interpolation=cv2.INTER_AREA)

            # Read in processed image to train array
            train[i] = np.array(resized_img)

            # Read in training labels
            train_labels[i] = np.int32(lines[i][1])

            # Block to visualize image processing
            # It stops the for loop on the line number range starts at so classifier arrays don't populate
            # Visualize hsv, s, thresh, cont_img and cropped_img
            # if(__debug__):
            #     cv2.imshow(Title_original, original_img)
            #     cv2.imshow(Title_processed, resized_img)
            #     key = cv2.waitKey()
            #     if key==27:    # Esc key to stop
            #         break

        # here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
        train_data = train.reshape((len(lines), self.height*self.width))/255
        train_data = train_data.astype(np.float32)

        ### Train classifier
        self.knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)
        self.get_logger().info('KNN Classifier Trained')
        

    def testImage(self, test_img):
        #print('testing KNN')
        k = 4
        # Convert to HSV and split
        hsv = cv2.cvtColor(test_img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        # Take a binary threshold
        ret,thresh = cv2.threshold(s,120,255,cv2.THRESH_BINARY)
        proc_img = thresh

        # Find contours if any
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Crop by largest contour
        if len(contours) != 0:
            #draw all contours in gray
            cont_img = thresh
            cv2.drawContours(cont_img, contours, -1, (100,100,100), 1)
            #find and draw a bounding rectangle around the biggest contour
            c = max(contours, key = cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(cont_img,(x,y),(x+w,y+h),(255,255,255),2)
            cropped_img = thresh[y:y+h, x:x+w]
            proc_img = cropped_img
            #print('Found a contour')

        resized_img = cv2.resize(proc_img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        #self.processed_frame = CvBridge().cv2_to_compressed_imgmsg(resized_img)
        self.processed_frame = CvBridge().cv2_to_compressed_imgmsg(test_img)

        test_img = np.array(resized_img)
        test_img = test_img.reshape((1,self.height*self.width))/255        
        test_img = test_img.flatten()
        test_img = test_img.astype(np.float32)     

        test = test_img.reshape(1, -1) 
        ret, results, neighbours, dist = self.knn.findNearest(test, k)

        if ret==1:
            return 'left'
        elif ret==2:
            return 'right'
        elif ret==3:
            return 'reverse'
        elif ret==4:
            return 'reverse'
        elif ret==5:
            return 'stop'
        else:
            print("\tneighbours: " + str(neighbours))
            print("\tdistances: " + str(dist))
            return 'empty'



    def _image_callback(self, CompressedImage):
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")


    def timer_callback(self):
        self.sign.data = self.testImage(self._imgBGR)
        self._sign_publisher.publish(self.sign)
        self._img_publisher.publish(self.processed_frame)





def main():
    rclpy.init() 
    # Create a class object to be used
    sign_reader = ReadSign()
    
    # Trigger callback processing
    rclpy.spin(sign_reader)
        
    #clean up and shutdown
    sign_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
