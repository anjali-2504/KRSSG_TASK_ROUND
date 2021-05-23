
import numpy as np
import cv2 as cv

img1= cv.imread("2.jpg")
dimensions = img1.shape[:2]
lower = np.array([0, 5, 50])
upper = np.array([179, 50, 255])
img2 = cv.cvtColor(img1, cv.COLOR_BGR2HSV)
mask = cv.inRange(img2, lower, upper)
output = cv.bitwise_and(img1, img1, mask=mask)
output_ = cv.cvtColor(output, cv.COLOR_BGR2GRAY)
img = output_.copy()


ret,thresh = cv.threshold(img,127,255,0)
contours,hierarchy = cv.findContours(thresh,  cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

obstacles=[]


for cnt in contours:
    (x, y, w, h) = cv.boundingRect(cnt)
    obstacles.append((x,y,h,w))
    cv.rectangle(img1, (x, y), (x+w, y+h), (0, 255, 0), 2)
    print(x,y,w,h)
 

cv.imshow("window",img1)
cv.waitKey(0)
cv.destroyAllWindows()
