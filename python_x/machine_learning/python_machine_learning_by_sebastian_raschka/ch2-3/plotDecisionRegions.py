#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

def plotDecisionRegions(X, y, classifier, resolution = 0.01):

    # Set up plot size. f = feature
    f0_min = X[:,0].min() - 1
    f0_max = X[:,0].max() + 1
    f1_min = X[:,1].min() - 1
    f1_max = X[:,1].max() + 1

    # Get matrices that represents each point in the plot so that we can feed
    # those points to the already-trained algorithm.
    f0_grid, f1_grid = np.meshgrid(
        np.arange(f0_min, f0_max, resolution),
        np.arange(f1_min, f1_max, resolution) )

    # The algorithm accepts N x M matrix so reshape the grid according to that
    # dimension.
    # ravel() turn a matrix to vector
    # grid_points = #points x 2.
    grid_points = np.array( [f0_grid.ravel(), f1_grid.ravel()] ).T

    # result: N x 1 vector for classification label.
    result = classifier.classify( grid_points )
    
    # Reshape the result to f0_grid by f1_grid
    result = result.reshape( f0_grid.shape )

    # Plot the result. The result (1 or -1) will be colored according to the
    # color map specified. The boundary between the two result regions will be
    # drawn with a line automatically by the contourf() function.
    colors = ['red', 'blue']
    listedColormap = ListedColormap( colors )
    plt.contourf( f0_grid,
                  f1_grid,
                  result,
                  alpha = 0.4,
                  cmap = listedColormap )
    plt.xlim( f0_grid.min(), f0_grid.max() )
    plt.ylim( f1_grid.min(), f1_grid.max() )

    # Above prints entire regions. Right now we want to graph the y's on top of
    # the plot. y is N x 1 vector. Plot by classification.
    markers = [ 'o', 'x' ]
    for idx, classification in enumerate( np.unique(y) ):
        # Get f0 that corresponds to the class label.
        f0_idx = X[ y == classification, 0 ]
        # Get f1 that corresponds to the class label.
        f1_idx = X[ y == classification, 1 ]

        plt.scatter( f0_idx,
                     f1_idx,
                     alpha = 0.8,
                     c = listedColormap(idx),
                     marker = markers[idx],
                     label = classification )
