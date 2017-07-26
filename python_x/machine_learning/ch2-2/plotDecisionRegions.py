#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# Draws two classification regions with different colors.
# X: N x M matrix. N = number of samples. M = number of features.
# y: N x 1 vector representing the true classification. N = number of samples.
# classifier: Classifier that is assumed to be already trained.
# resolution: Resolution of the plot
def plotDecisionRegions(X, y, classifier, resolution = 0.01):
    markers = ( 's', 'x', 'o', '^', 'v' )
    colors = ('red', 'blue', 'lightgreen', 'gray', 'cyan' )

    # Set up color map based on the number of classifications.
    listedColormap = ListedColormap( colors[:len(np.unique(y))] )

    # Set up the plot boundaries
    x1_min, x1_max = X[:, 0].min() - 1, X[:, 0].max() + 1
    x2_min, x2_max = X[:, 1].min() - 1, X[:, 1].max() + 1

    # Set up the grid
    # x1_grid: N x M matrix where all rows are same, but columns are according
    #          to the [x1_min, x1_max]
    # x2_grid: N x M matrix where all columns are same, but rows are according
    #          to the [x2_min, x2_max]
    x1_grid, x2_grid = np.meshgrid( np.arange(x1_min, x1_max, resolution),
                                    np.arange(x2_min, x2_max, resolution) )

    # Predict each cell in the feature 1 (x1) and feature 2 (x2) matrix x1 x x2.
    # User vectorization to predict all cells at once.
    # Somehow, I do not need to transpose np.arra() output (unlike the
    # example).
    result = classifier.predict( np.array([x1_grid.ravel(), x2_grid.ravel()]) )
    result = result.reshape( x1_grid.shape )

    # Plot the pretty colored classification boundary and regions.
    plt.contourf( x1_grid, 
                  x2_grid, 
                  result, 
                  alpha = 0.4, 
                  cmap = listedColormap )

    plt.xlim( x1_grid.min(), x1_grid.max() )
    plt.ylim( x2_grid.min(), x2_grid.max() )

    # Plot each predicted result points. Plot all points with same kind and
    # then move to the next kind.
    for idx, classification in enumerate( np.unique(y) ):
        plt.scatter( x = X[y == classification, 0], # feature 1
                     y = X[y == classification, 1], # feature 2 
                     alpha = 0.8,
                     c = listedColormap(idx),
                     marker = markers[idx],
                     label = classification )
