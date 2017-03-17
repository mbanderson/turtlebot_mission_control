from scipy import ndimage
import numpy as np

if __name__ == '__main__':
    test = np.array([[1,1,1,0,0,0,0],
                    [0,1,1,0,0,0,0],
                     [0,0,0,0,0,0,0],
                     [0,0,0,0,0,1,1],
                    [0,0,0,0,0,1,1],
                    [1,1,0,0,0,0,0]])
    labels = ndimage.label(test)[0]
    print labels

    groups = range(1,np.amax(labels)+1)

    centers = ndimage.measurements.center_of_mass(test, labels, groups)
    print centers
