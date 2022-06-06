import cv2
import numpy as np
import time

t0 = time.perf_counter()
img = cv2.imread('coco21.jpg')
height, width, channels = img.shape
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#edges = cv2.Sobel(img, cv2.CV_64F, 0, 1)
edges = cv2.Canny(gray,200,300,apertureSize = 5)
#cv2.imshow('edges',edges)

hough_thresh = 1000

while hough_thresh > 100:
    try:
        lines = cv2.HoughLines(edges,1,np.pi/180,hough_thresh)
        horizon = img.copy()
        for line in lines:
            rho,theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            y_avg = int((y1+y2)/2)
            horizon = cv2.line(horizon,(0,y_avg),(width-1,y_avg),(0,0,255),2)
        break
    except:
        hough_thresh = hough_thresh - 50
        continue

crop = img[y_avg:height, 0:width]
crop_h, crop_w, crop_ch = crop.shape
canvas = np.zeros((crop_h,crop_w,1), dtype=np.uint8)
ybottom = 0

saliency = cv2.saliency.StaticSaliencySpectralResidual_create()
(success, saliencyMap) = saliency.computeSaliency(crop)
saliencyMap = (saliencyMap * 255).astype("uint8")
cv2.imshow('saliency map',saliencyMap)

ret, crop_thresh = cv2.threshold(saliencyMap, 70, 255, cv2.THRESH_BINARY)
#cv2.imshow('Threshold',crop_thresh)

result = img.copy()
contours = cv2.findContours(crop_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
for cntr in contours:
    x,y,w,h = cv2.boundingRect(cntr)
    cv2.rectangle(canvas, (x, y), (x+w, y+h), (255, 255, 255), -1)
    if y+h > ybottom:
        ybottom = y+h

contours = cv2.findContours(canvas, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = contours[0] if len(contours) == 2 else contours[1]
for cntr in contours:

    x,y,w,h = cv2.boundingRect(cntr)
    if w > 10 and h > 10:
        if y+h >= ybottom:
            cv2.rectangle(result, (x, y + height - crop_h), (x + w, y + h + height - crop_h), (0, 0, 255), 2)

            koor = (x + w)/2
            koor_norm = koor/width

        else:
            cv2.rectangle(result, (x, y + height - crop_h), (x + w, y + h + height - crop_h), (255, 255, 255), 2)
    else:
        continue

print(koor_norm)

cv2.imshow('asli',img)
#cv2.imshow('crop',crop)
#cv2.imshow('horizon',horizon)

cv2.imshow('Hasil',result)

t1 = time.perf_counter() - t0
fps = 1/t1
#print("fps :", fps)

cv2.waitKey(0)



