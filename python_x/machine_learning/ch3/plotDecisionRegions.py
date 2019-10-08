#!/usr/bin/python

# For the original location of this entire fire, see Ch3-2.
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import numpy as np

# X: n x m matrix for input samples. n = number of samples. m = number of features.
def plotDecisionRegions( X,
                         y,
                         classifier,
                         test_idx = None,
                         resolution = 0.02 ) :
    
    # Generate the plot cells so that we can classify them.
    x1_min = X[:,0].min() - 1
    x1_max = X[:,0].max() + 1
    x2_min = X[:,1].min() - 1
    x2_max = X[:,1].max() + 1
    varying_cols, varying_rows = np.meshgrid( np.arange(x1_min, x1_max, resolution),
                                              np.arange(x2_min, x2_max, resolution) )

    # z = prediction
    z = classifier.predict( np.array([varying_cols.ravel(), varying_rows.ravel()]).T )
    z = z.reshape( varying_cols.shape )
   
    # Plot the decision boundary
    colors = ( 'red', 'blue', 'lightgreen', 'gray', 'cyan' )
    colorMap = ListedColormap( colors[:len(np.unique(y))] )
    plt.contourf( varying_cols, varying_rows, z, alpha = 0.4, cmap = colorMap )
    plt.xlim( varying_cols.min(), varying_cols.max() )
    plt.ylim( varying_rows.min(), varying_rows.max() )

    # Plot all samples.
    markers = ( 's', 'x', 'o', '^', 'v' )
    for idx, label in enumerate( np.unique(y) ):
        plt.scatter( x = X[ y == label, 0],   # feature 1
                     y = X[ y == label, 1],   # feature 2
                     alpha = 0.8, 
                     c = colorMap(idx),
                     marker = markers[idx], 
                     label = label )

    # Highlight the test samples that were chosen among all the samples.
    if test_idx :
        X_test = X[ test_idx, : ]
        plt.scatter( X_test[:, 0],   # feature 1
                     X_test[:, 1],   # feature 2
                     c = '',
                     alpha = 1.0,
                     linewidth = 1,
                     marker = 'o',
                     s = 55,
                     label = 'test set' )
