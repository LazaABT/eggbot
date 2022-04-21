'''
For functions concerning color manipulation

'''

# Pen colors
# white, yellow, green, blue, red
penColors = [(255,255,255), (0, 0, 0), (235, 64, 52), (0, 255, 208), 
             (51, 255, 0), (0, 76, 255), (255, 0, 0), (178, 214, 200)]


def sqDist(a, b):
    # 3d euclidean distance squared
    return (a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2

def closestColor(color):
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
    if(minDist == 100000): return -1
    
    return bestColor

def convertImage(im):
    ''' Retruns image with colors replaced with their closest counterpart
        in the penColors array
    '''
    newIm = im[:]
    for i in range(im.shape[0]):
        for j in range(im.shape[1]):
            newIm[i][j] = closestColor(im[i][j])
        
    return newIm

