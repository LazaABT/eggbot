import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle
import random as rng

from color import loadColors
from color import convertImage

#  ------ CONFIG ------
imgFolder = r"test_images/"
fileName = r"colorBar.jpg"

outFolder = r"out_images/"
outFile = r"shOut.png"

colorFile = "colorref.pickle"

SAVE_IMAGE = False

INTERP_FACTOR = 10
KERNEL_SIZE = 10
DIST_THRESH = 2
CONTOURS = False
#  ------ /CONFIG ------


# Filter contour
def circ_convolve(signal, ker):
    return np.real(np.fft.ifft( np.fft.fft(signal)*np.fft.fft(ker, len(signal))))


# Calculate distance of point(s) from line made by two points
def distFromLine(p1, p2, points):
    # Returns array of distances from points to line defined by p1 and p2
    return np.abs(np.cross(p2-p1, p1-points)/np.linalg.norm(p2-p1))

def maxDist(p1, p2, points):
    return np.max(distFromLine(p1, p2, points))




# Load colors
penColors = loadColors(colorFile)

# Read image
im = cv2.imread(imgFolder+fileName)
im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)   # BGR -> RGB
im = im.astype(float)

# Generate image with colors from palette
newIm, minIndexes = convertImage(im.copy(), penColors)

if CONTOURS:
    
    # Generate mask from conversion results for one color
    mask = (minIndexes==0).astype(float)
    new = mask.astype(np.uint8)
    
    # Find contours
    contours, hierarchy = cv2.findContours(new, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(im, contours, -1, (0,255,0), 1)
    
    #Extract xs and ys and plot
    xs = []
    ys = []
    for cnt in contours:
        x = cnt[:, 0, 0]
        y = cnt[:, 0, 1]
        xs.append(x)
        ys.append(y)
    
    
    # Choose contour and connect the ends
    x = xs[2]
    y = ys[2]
    x = np.concatenate((x, [x[0]]))
    y = np.concatenate((y, [y[0]]))
    
    #Find s
    xdiff = np.diff(x)
    ydiff = np.diff(y)
    s = np.cumsum(np.sqrt(xdiff**2+ydiff**2))
    s = np.concatenate(([0], s))
    
    #Interpolate
    sNew = np.linspace(s[0], s[-1]-1/INTERP_FACTOR, int(s[-1]*INTERP_FACTOR))
    xNew = np.interp(sNew, s, x)
    yNew = np.interp(sNew, s, y)
    
    kernel = np.ones(INTERP_FACTOR*KERNEL_SIZE)
    kernel = kernel/np.sum(kernel)
    
    xFilt = circ_convolve(xNew, kernel)
    yFilt = circ_convolve(yNew, kernel)
    

    
    #Extract longest lines
    p1 = np.array([xFilt[0], yFilt[0]])         # First point of line
    points = np.array([p1])                     # Points between endpoints of line
    lines = np.array([p1])                      # Points making up resulting lines
    for i in range(1, len(xFilt)):
        p2 = np.array([xFilt[i], yFilt[i]])     # Second point of line
        
        if maxDist(p1, p2, points) < DIST_THRESH:
            # Extend line
            points = np.concatenate((points, [p2]))
        else:
            # Move on to next line
            # Make last between point new line endpoint
            lines = np.concatenate((lines, [points[-1]]))
            p1 = points[-1].copy()  #New line start is p2
            points = np.array([p2])
    # Add last point to lines
    lines = np.concatenate((lines, [p2]))

    # Draw contours
    drawing = np.ones((new.shape[0], new.shape[1], 3), dtype=np.uint8)*255
    #drawing = im.copy()
    for i in range(len(contours)):
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        cv2.drawContours(drawing, contours, i, color, 1, cv2.LINE_8, hierarchy, 0)

f, a = plt.subplots(1, 1)

a.imshow(newIm.astype(int))

# x, y = np.array(list(zip(*lines)))
# plt.plot(x, y, 'k')
# plt.plot(xFilt, yFilt, 'k')
   
plt.show()



if(SAVE_IMAGE):
    img = np.uint8(drawing)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # BGR <- RGB
    cv2.imwrite(outFolder+outFile, img)

