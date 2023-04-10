import cv2 as cv
import numpy as np
from PerspectiveTransformation import *
srcRegion = np.float32([(120, 300),     # top-left
                        (20, 700),     # bottom-left
                        (1260, 700),    # bottom-right
                        (1160, 300)])    # top-right
dstRegion = np.float32([(0, 0),
                        (0, 720),
                        (1260, 720),
                        (1260, 0)])
transform = PerspectiveTransformation(srcRegion, dstRegion)


def ResizeWithAspectRatio(image, width=None, height=None, inter=cv.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv.resize(image, dim, interpolation=inter)


def showWindows(image, name, x, y, width=None):
    scale_image = ResizeWithAspectRatio(image, width=width)
    cv.imshow(name, scale_image)
    cv.moveWindow(name, x, y)


# Read Image
img = cv.imread("../media/road_2.jpg")
rep = img.copy()
cv.circle(rep, (120, 300), radius=5, color=(0, 255, 0), thickness=5)
cv.circle(rep, (20, 700), radius=5, color=(0, 255, 0), thickness=5)
cv.circle(rep, (1260, 700), radius=5, color=(0, 255, 0), thickness=5)
cv.circle(rep, (1160, 300), radius=5, color=(0, 255, 0), thickness=5)
showWindows(rep, "source", 20, 20, 800)
print(img.shape)
img = transform.forward(img)
showWindows(img, "forward", 820, 20, 800)

# Convert to binary
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# showWindows(gray, "gray", 820, 20, 800)

# Apply canny
edges = cv.Canny(gray, 75, 80)
lines = cv.HoughLinesP(edges, 1, np.pi/360, 20, maxLineGap=30)
# print(lines)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

showWindows(edges, "canny", 20, 500, 800)
img = transform.backward(img)
showWindows(img, "houghLines", 820, 500, 800)
while True:
    key = cv.waitKey(25)
    if key == 27:
        break
