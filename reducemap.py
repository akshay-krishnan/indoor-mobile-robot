import cv2
import numpy as np


img = cv2.imread('map4.pgm')
imga2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
h, w = imga2.shape
ret,img2 = cv2.threshold(imga2, 120, 255, cv2.THRESH_BINARY)
img4 = cv2.bitwise_not(img2)
cv2.imshow('win1', img2)
fac = 8

img3 = np.ndarray((h/fac, w/fac))
i = 0
j = 0

for i in range(0, h/fac):
    for j in range(0, w/fac):
        print i,j, np.count_nonzero(img4[fac*i:(fac*i)+fac, j*fac:(j*fac)+fac]), img4[fac*i:(fac*i)+fac, j*fac:(j*fac)+fac]
        if np.count_nonzero(img4[fac*i:(fac*i)+fac, j*fac:(j*fac)+fac]) > 0:
            img3[i][j] = 0
        else:
            img3[i][j] = 255


cv2.imshow('win', img3)

cv2.imwrite('reduced.pgm', img3)


