import cv2
import numpy as np
 
 
def on_change(value):
    print(value/100)
 
 
blank_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8)
 
windowName = 'sliders'
 
cv2.imshow(windowName, blank_image)
cv2.createTrackbar('slider', windowName, 0, 100, on_change)
 
cv2.waitKey(0)
cv2.destroyAllWindows()