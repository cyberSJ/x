#!/usr/bin/python
# Tests the Adaline macihne learning algorithm.
# Usage ./test.py

from Adaline import Adaline
from AdalineOnline import AdalineOnline
from plotDecisionRegions import plotDecisionRegions
# File can be downloaded from a website through pandas.
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

csv_file = pd.read_csv( 'https://archive.ics.uci.edu/ml/machine-learning-databases/iris/iris.data',
                        header = None )

print csv_file.tail()

# Classification label
y = csv_file.iloc[0:100, 4].values
y = np.where( y == "Iris-setosa", -1, 1 )

# Training data
X_raw = csv_file.iloc[0:100, [0,2]].values

# Use standardization technique for pre-processing feature scaling.
X = np.copy(X_raw)
X[:,0] = (X_raw[:,0] - X_raw[:,0].mean()) / X_raw[:,0].std()
X[:,1] = (X_raw[:,1] - X_raw[:,1].mean()) / X_raw[:,0].std()

# fig is unused.
fig, ax = plt.subplots( nrows = 1, 
                        ncols = 2, 
                        figsize=(8, 4) )

# In Python, you can call member function directly after
# construction.
adalineSteep = Adaline( learning_rate = 0.01,
                        iteration = 10 ).learn( X, y )

ax[0].plot( range(1, len(adalineSteep.costs_) + 1),
            np.log10(adalineSteep.costs_),
            marker = 'o' )
ax[0].set_xlabel( '# iteration' )
ax[0].set_ylabel( 'log10( sum-squared error )' )
ax[0].set_title( 'Adaline Steep' )

adalineGradual = Adaline( learning_rate = 0.00001,
                          iteration = 10 ).learn( X, y )

# iteration vs. cost
ax[1].plot( range(1, len(adalineGradual.costs_) + 1),
            adalineGradual.costs_,
            marker = 'o' )
ax[1].set_xlabel( '# iteration' )
ax[1].set_ylabel( 'sum-squared error' )
ax[1].set_title( 'Adaline Gradual' )

plt.show()


# Plot the regions and the training points. This plot ultimately shows how
# the training went.
adalineJustRight = Adaline( learning_rate = 0.01,
                            iteration=15 )

adalineJustRight.learn( X, y )

# Calling plotDecisionRegions() now plots thing to the left subplot?
plotDecisionRegions( X, y, adalineJustRight )
plt.title( 'Adaline Just Right' )
plt.xlabel( 'sepal length [standardized]' )
plt.ylabel( 'pteal length [ standadized]' )
plt.legend( loc='upper left' )
plt.show()

# Plot the cost in the right subplot.
plt.plot( range(1, len(adalineJustRight.costs_) + 1),
          adalineJustRight.costs_,
          marker = 'o' )
plt.xlabel( '# iteration' )
plt.ylabel( 'sum squared error' )
plt.show()

# New section - Plot the stochastic gradient descent version of the Adaline
# algorithm.
adalineStochastic = AdalineOnline( learning_rate = 0.01,
                                   iterations = 15,
                                   random_state_seed = 1 )

adalineStochastic.stochasticGradientDescent( X, y )

plotDecisionRegions( X, y, classifier = adalineStochastic )
plt.title( 'Adaline Stochastic Gradient Descent' )
plt.xlabel( 'sepal length [standardized]' )
plt.ylabel( 'pteal length [standardized]' )
plt.legend( loc = 'upper left' )
plt.show()

# Plot the average costs for each iteration over all the data.
print "sung avg_cost_ length: %d" % len(adalineStochastic.avg_costs_)
plt.plot( range(1, len(adalineStochastic.avg_costs_) + 1),
          adalineStochastic.avg_costs_,
          marker = 'o' )
plt.xlabel( '# iterations' )
plt.ylabel( 'Average cost' )
plt.show()
