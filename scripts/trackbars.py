import cv2
import imutils
import numpy as np

def callback(x):
    pass

cap = cv2.VideoCapture(0)
cv2.namedWindow('image')

# initial track bar limits
ilowH = 0
ihighH = 7

ilowS = 220
ihighS = 255

ilowV = 182
ihighV = 255

erode = 0
dilate = 0

# create trackbars for color change
cv2.createTrackbar('lowH','image',ilowH,255,callback)
cv2.createTrackbar('highH','image',ihighH,255,callback)

cv2.createTrackbar('lowS','image',ilowS,255,callback)
cv2.createTrackbar('highS','image',ihighS,255,callback)

cv2.createTrackbar('lowV','image',ilowV,255,callback)
cv2.createTrackbar('highV','image',ihighV,255,callback)

cv2.createTrackbar('erode','image',erode,10,callback)
cv2.createTrackbar('dilate','image',dilate,10,callback)



while(1):
    # get trackbar positions
    ilowH = cv2.getTrackbarPos('lowH', 'image')
    ihighH = cv2.getTrackbarPos('highH', 'image')
    ilowS = cv2.getTrackbarPos('lowS', 'image')
    ihighS = cv2.getTrackbarPos('highS', 'image')
    ilowV = cv2.getTrackbarPos('lowV', 'image')
    ihighV = cv2.getTrackbarPos('highV', 'image')
    erode = cv2.getTrackbarPos('erode', 'image')
    dilate = cv2.getTrackbarPos('dilate', 'image')
    # Read frame
    ret, frame = cap.read()
    frame = imutils.resize(frame, width=600)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv', hsv)
    lower_hsv = np.array([ilowH, ilowS, ilowV])
    higher_hsv = np.array([ihighH, ihighS, ihighV])
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
    mask = cv2.erode(mask, None, iterations=erode)
    mask = cv2.dilate(mask, None, iterations=dilate)
    cv2.imshow('mask', mask)
    cv2.imshow('frame', frame)
    print (ilowH, ilowS, ilowV)
    print (ihighH, ihighS, ihighV)
    if(cv2.waitKey(1) & 0xFF == ord('q')):
        break

cv2.destroyAllWindows()
cap.release()