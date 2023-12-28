import cv2
import numpy as np
img = cv2.imread("../media/road.jpg")
cv2.imshow("source", img)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow("gray", gray)
edges = cv2.Canny(gray, 80, 100)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30, maxLineGap=250)
# print(lines)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

cv2.imshow("edges", edges)
# cv2.imshow("image", img)
while True:
    key = cv2.waitKey(25)
    if key == 27:
        break
