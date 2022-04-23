'''
For functions concerning color manipulation

'''

import numpy as np
import pickle

# Pen colors
#penColors = [[255, 255, 255], [213, 192, 101], [79, 150, 180], [151, 160, 159], [172, 129, 78], [69, 137, 96], [171, 62, 68], [131, 76, 81], [47, 70, 150], [71, 71, 73]]
#colorNames = ['white', 'yellow', 'cyan', 'gray', 'orange', 'green', 'red', 'brown', 'blue', 'black']


def loadFile(file):
    #Loads pen colors from pickle file
    with open(file, "rb") as f:
        obj = pickle.load(f)
    return obj

def saveFile(file, obj):
    with open(file, "wb") as f:
        pickle.dump(obj, f)
    print("Wrote to file", file)

def sqDist(a, b):
    # 3d euclidean distance squared
    return (a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2

def closestColor(color, penColors):
    ''' Returns pen color best matching given color
    color - array of 3 elements (r, g, b)
    '''
    bestColor = penColors[0]
    minDist = 100000
    for i in range(len(penColors)):
        dist = sqDist(color, penColors[i])
        if(dist < minDist):
            minDist = dist
            bestColor = penColors[i]
    
    return bestColor


def convertImage(im, penColors, mults=None):
    ''' Retruns image with colors replaced with their closest counterpart
        in the penColors array
    '''

    if not mults:
        mults = [1]*len(penColors)
        
    
    newIm = im
    errIm = np.zeros((im.shape[0], im.shape[1], len(penColors)))
    for i in range(len(penColors)):
        errIm[:,:,i] = np.sum((im-penColors[i])**2, 2)*mults[i]
    
    minIndexes = np.argmin(errIm, 2)
    
    
    for cind in range(len(penColors)):
        for i in range(3):
            
            newIm[:, :, i][minIndexes==cind] = penColors[cind][i]
    
    return (newIm, minIndexes)


def generatePaletteImage(penColors):
    # Color reference image
    ciw = 100
    cih = 20
    pcl = len(penColors)
    colorImage = np.ones((cih, ciw, 3))
    w = ciw//pcl
    for i in range(pcl):
        colorImage[:, i*w:(i+1)*w] = np.array(penColors[i])/255
    return colorImage