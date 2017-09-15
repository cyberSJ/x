#!/usr/bin/env python
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import numpy as np

# X: n x m matrix representing a set of samples. n: number of samples. m: number
# of features.
def plotDecisionRegions( X,
                         y,
                         classifier,
                         resolution = 0.02 ):

    # Set up the markers and colors.
    markers = ( 's', 'x', 'o', '^', 'v' )
    colors = ( 'red', 'blue', 'lightgreen', 'gray', 'cyan' )
    cmap = ListedColormap( colors[:len(np.unique(y))] )


    # Determine the data limit in order to set the graph boundary.
    x1_min = X[:,0].min() - 1
    x1_max = X[:,0].max() + 1
    x2_min = X[:,1].min() - 1
    x2_max = X[:,1].max() + 1
    
    # Generate the graph granularity points.
    # xx1 is now column-varying matrix. Kinda acts like x-axis on a graph.
    # xx2 is now a row-varying matrix. Kinda acts like y-axis on a graph.
    # But still, I feel like xx1 should be the 1st-D-varying, i.e. row-varying
    # elements.... But meshgrid behaves in the opposite way.
    xx1, xx2 = np.meshgrid( np.arange(x1_min, x1_max, resolution),
                            np.arange(x2_min, x2_max, resolution) )

    # Classify each graph point.
    classified = classifier.pred( np.array( [xx1.ravel(), xx2,ravel()] ).T )
    classified = classified.reshape( xx1.shape )

    plt.contourf( xx1, 
                  xx2, 
                  classified, 
                  alpha = 0.4,
                  cmap = cmap )

    plt.xlim( xx1.min(), xx1.max() )
    plt.ylim( xx2.min(), xx2.max() )

    # Plot the actual samples in X. 
    for idx, lab in enumerate( np.unique(y) ):
        plt.scatter( X[y == lab, 0],
                     X[y == lab, 1],
                     alpha - 0.8,
                     c = cmap[idx],
                     marker = markers[idx],
                     label = lab )
