# Lab 1, AE7785
# Richard Agbeyibor & Bhushan Pawaskar
# Find, bound & mark ball

#import modules
import cv2
import numpy as np

# define a video capture object
vid = cv2.VideoCapture(0)


while(True):   
    ret, img = vid.read()
   
    # to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   
    # make a 5x5 kernel
    kernel = np.ones((5, 5), np.uint8)

    # blur the image
    blur = cv2.blur(gray,(5,5))

    # take a binary threshold
    ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_TOZERO)

    #Opening and closing
    dilate1 = cv2.dilate(thresh, kernel, iterations=1)
    erode1 = cv2.erode(dilate1, kernel, iterations=2)
    dilate2 = cv2.dilate(erode1, kernel, iterations=1)

    #optimised parameters for hough circles
    circles = cv2.HoughCircles(dilate2, cv2.HOUGH_GRADIENT, dp=2, minDist=480, param1=100, param2=50, minRadius=50, maxRadius=200)
    
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            #print coordinates and radius
            print(i)
            #draw the circle
            cv2.circle(img,(i[0], i[1]),i[2],(0,255,255),2)
            #draw circle center
            cv2.circle(img,(i[0], i[1]),2,(0,0,255),2)
    except TypeError:
        pass
    cv2.imshow('image', img)
    #cv2.imshow('debug', dilate2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the capture
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()