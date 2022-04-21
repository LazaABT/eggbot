import cv2
import matplotlib.pyplot as plt
import numpy as np

from color import convertImage
from color import penColors

imgFolder = r"test_images/"
fileName = r"squidward.png"

outFolder = r"out_images/"
outFile = r"squidOut.png"

ciw = 100
cih = 20
pcl = len(penColors)
colorImage = np.ones((cih, ciw, 3))
w = ciw//pcl
for i in range(pcl):
    colorImage[:, i*w:(i+1)*w] = np.array(penColors[i])/255



im = cv2.imread(imgFolder+fileName)
img = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)   # BGR -> RGB
img = img.astype(float)
newIm = convertImage(img)
img = newIm.astype(int)

f, a = plt.subplots(1, 2)
a[0].imshow(img)
a[1].imshow(colorImage)

img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)   # BGR <- RGB
cv2.imwrite(outFolder+outFile, img)



