#!/usr/bin/env python3

import cv2
import sys
import csv
import time
import numpy as np

### Load training images and labels

imageDirectory = './2022Fimgs/'

if(__debug__):
    Title_original = 'Original Image'
    Title_processed = 'Processed Image'
    cv2.namedWindow( Title_original, cv2.WINDOW_AUTOSIZE )

with open(imageDirectory + 'train.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)
width = int(64/4)
height = int(48/4)
train = np.zeros((len(lines), height, width))
train_labels = np.zeros((len(lines)), dtype=int)

for i in range(0, len(lines)):
    # Load original image
    original_img = cv2.imread(imageDirectory+lines[i][0]+".png",1)
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

    resized_img = cv2.resize(proc_img, (width, height), interpolation=cv2.INTER_AREA)

    # This line reads in processed image to train array
    train[i] = np.array(resized_img)

    # This line reads in training labels
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

train_data = train.reshape((len(lines), height*width))/255
train_data = train_data.astype(np.float32)


### Train classifier
knn = cv2.ml.KNearest_create()
knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)
print(train_data)
print(train_data.shape)

imageDirectory = './2022Fimgs/'

### Run test images
with open(imageDirectory + 'test.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

test = np.zeros((len(lines), height, width))
test_label = np.zeros((len(lines)), dtype=int)
correct = 0.0
confusion_matrix = np.zeros((6,6))

k = 4

for i in range(len(lines)):
    original_img = cv2.imread(imageDirectory+lines[i][0]+".png",1)
    # test_img = np.array(cv2.resize(cv2.imread(imageDirectory+lines[i][0]+".png",0),(width,height)))
    
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

    resized_img = cv2.resize(proc_img, (width, height), interpolation=cv2.INTER_AREA)

    # This line reads in processed image to train array
    test[i] = np.array(resized_img)
    test_label[i] = np.int32(lines[i][1])
    
    # Block to visualize image processing
    # It stops the for loop on the line number range starts at so classifier arrays don't populate
    # Visualize hsv, s, thresh, cont_img and cropped_img
    # if(__debug__):
    #     cv2.imshow(Title_original, original_img)
    #     cv2.imshow(Title_processed, resized_img)
    #     key = cv2.waitKey()
    #     if key==27:    # Esc key to stop
    #         break

    

# test_img = test_img.flatten().reshape(1, width*height)
test_img = test.reshape((len(lines),height*width))/255
test_img = test_img.astype(np.float32)

print(test_img[0])
print(test_img.shape)      


for i in range(len(lines)):
    test = test_img[i].reshape(1, -1) 
    ret, results, neighbours, dist = knn.findNearest(test, k)

    if test_label[i] == ret:
        print(str(lines[i][0]) + " Correct, " + str(ret))
        correct += 1
        confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
    else:
        confusion_matrix[test_label[i]][np.int32(ret)] += 1
        
        print(str(lines[i][0]) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        print("\tneighbours: " + str(neighbours))
        print("\tdistances: " + str(dist))



print("\n\nTotal accuracy: " + str(correct/len(lines)))
print(confusion_matrix)
