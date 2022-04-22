import cv2
import matplotlib.pyplot as plt
import numpy as np

from color import loadColors
from color import convertImage

#  ------ CONFIG ------
imgFolder = r"test_images/"
fileName = r"surprised_pikachu.png"

outFolder = r"out_images/"
outFile = r"wheelOut.png"

colorFile = "colorref.pickle"

SAVE_IMAGE = False


penColors = loadColors(colorFile)


# Color reference image
ciw = 100
cih = 20
pcl = len(penColors)
colorImage = np.ones((cih, ciw, 3))
w = ciw//pcl
for i in range(pcl):
    colorImage[:, i*w:(i+1)*w] = np.array(penColors[i])/255


#Read image and convert to pen colors
im = cv2.imread(imgFolder+fileName)
im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)   # BGR -> RGB
im = im.astype(float)
newIm = convertImage(im.copy(), penColors)
img = newIm.astype(int)

#Show image and color pallette
f, a = plt.subplots(2, 1)
a[0].imshow(im.astype(int))
a[1].imshow(img)



if(SAVE_IMAGE):
    img = np.uint8(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # BGR <- RGB
    cv2.imwrite(outFolder+outFile, img)



