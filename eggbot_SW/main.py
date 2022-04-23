import cv2
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = (10,5)
import numpy as np
import sys

from matplotlib.widgets import CheckButtons
from matplotlib.widgets import Button

from color import loadFile
from color import saveFile
from color import convertImage

#  ------ CONFIG ------
imFolder = r"test_images/"
imFile = r"lines.png"

outFolder = r"out_images/"
outFile = r"shOut.png"

contFolder = "contour_files/"
contFile = "_cnts.pickle"

#colorFile = "colorref.pickle"
colorFile = "colorsNamed.pickle"


SAVE_IMAGE = False
CONTOURS = True
ERODE = True

DIST_THRESH = 2
MIN_CONTOUR_LENGTH = 50

#  ------ /CONFIG ------

# Calculate distance of point(s) from line made by two points
def distFromLine(p1, p2, points):
    # Returns array of distances from points to line defined by p1 and p2
    return np.abs(np.cross(p2-p1, p1-points)/np.linalg.norm(p2-p1))

def maxDist(p1, p2, points):
    return np.max(distFromLine(p1, p2, points))

def lineifyContour(x, y):
    # Returns array of points
    
    # If contour too short returns none
    xdiff = np.diff(x)
    ydiff = np.diff(y)
    s = np.sum(np.sqrt(xdiff**2+ydiff**2))
    if s<MIN_CONTOUR_LENGTH:
        return None
    
    #Extract longest lines
    p1 = np.array([x[0], y[0]])         # First point of line
    points = np.array([p1])                     # Points between endpoints of line
    lines = np.array([p1])                      # Points making up resulting lines
    for i in range(1, len(x)):
        p2 = np.array([x[i], y[i]])     # Second point of line
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
    return lines


def filterContours(contours):
    # Returns contsFilt - array of contours (contour = array of points)
    contsFilt = []
    print(len(contours))
    
    for cnt in contours:
        if(len(cnt) < 2): continue
        x = cnt[:, 0, 0]
        y = cnt[:, 0, 1]

        x = np.concatenate((x, [x[0]]))
        y = np.concatenate((y, [y[0]]))
                
        #Extract longest lines
        lines = lineifyContour(x, y)
        if not(lines is None):
            contsFilt.append(lines)
                
    return contsFilt


def checkChanged(label):
    i = colorNames.index(label)
    colorsShown[i] = not colorsShown[i]
    plotConts(conts, colorsShown, penColors)
    
    
def plotConts(conts, colorsShown, penColors):
    a.clear()
    if showIm: a.imshow(newIm.astype(int)) 
    for i, cnts in enumerate(conts):
        if colorsShown[i]:
            for lines in cnts:
                x, y = np.array(list(zip(*lines)))
                a.plot(x, y, 'k')
                #a.plot(x, y, color=np.array(penColors[i])/255)
    
    a.set_ylim([height, 0])
    a.set_xlim([0, width])
    f.canvas.draw()
    
    
def dumpButtonClicked(e):
    # Dump contours into file
    d = {}
    for i, cnts in enumerate(conts):
        d[colorNames[i]] = cnts
    # Assumes imFile has 3 letter extension (.jpg, .png)
    saveFile(contFolder+imFile[:-4]+contFile, d)

def showButtonClicked(e):
    global showIm
    showIm = not showIm
    plotConts(conts, colorsShown, penColors)




# Load colors
(penColors, colorNames) = loadFile(colorFile)
colorsShown = [False]*len(penColors)

# UI Variables
showIm = False

# Read image
try:
    im = cv2.imread(imFolder+imFile)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)   # BGR -> RGB
    im = im.astype(float)
except Exception:
    print("ERROR: File does not exist.")
    sys.exit()



height, width, _ = im.shape

# Generate image with colors from palette
newIm, minIndexes = convertImage(im.copy(), penColors)

if CONTOURS:
    
    drawing = []
    conts = []
    for colorIndex in range(len(penColors)):
        # Generate mask from conversion results for one color
        mask = (minIndexes==colorIndex).astype(float)
        if ERODE: mask = cv2.erode(mask, np.ones((3, 3))/9)
        mask = mask.astype(np.uint8)
        
        # Find contours
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        conts.append(filterContours(contours))
        
        # Draw contours on drawing
        drawing.append(np.ones((im.shape[0], im.shape[1], 3), dtype=np.uint8)*255)  
        cv2.drawContours(drawing[colorIndex], contours, -1, penColors[colorIndex], 1, cv2.LINE_8)
        # for i in range(len(contours)):
        #     color = list(map(int, np.random.rand(3,)*255))
        #     cv2.drawContours(drawing, contours, i, color, 1, cv2.LINE_8, hierarchy, 0)
        


# Drawing
f, a = plt.subplots(1, 1)

if CONTOURS:
    # a.imshow(drawing.astype(int))
    plotConts(conts, colorsShown, penColors)
    plt.show()
    
    plt.subplots_adjust(left=0.35)
    
    # CheckButtons
    cax = plt.axes([0.05, 0.2, 0.2, 0.6])
    check = CheckButtons(cax, colorNames, colorsShown)
    check.on_clicked(checkChanged)
    # Dump Button
    dbax = plt.axes([0.05, 0.05, 0.1, 0.1])
    dumpButton = Button(dbax, 'Dump to file')
    dumpButton.on_clicked(dumpButtonClicked)
    # Show image button
    sbax = plt.axes([0.15, 0.05, 0.1, 0.1])
    showButton = Button(sbax, 'Show image')
    showButton.on_clicked(showButtonClicked)
    
else:
    a.imshow(newIm.astype(int))
   


if(SAVE_IMAGE):
    img = np.uint8(drawing)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # BGR <- RGB
    cv2.imwrite(outFolder+outFile, img)

