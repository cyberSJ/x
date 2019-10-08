#!/usr/bin/env python
# From Ch3-6 & 7.
# Conclusion: the entropy impurity function seems to penalize more when the set 
# is impure.

import matplotlib.pyplot as plt
import numpy as np

def gini(p):
    return (p) * (1-p) + (1 - p) * (1 - (1-p))

def entropy(p):
    return -p * np.log2(p) - (1-p) * np.log2(1-p)

def error(p):
    return 1 - np.max([p, 1-p])

# Input
x = np.arange(0.0, 1.0, 0.01)

# Output based on the different impurity function.
gin = [gini(p) if p != 0 else None for p in x]

ent = [entropy(p) if p != 0 else None for p in x]
scaled_ent = [e * 0.5 if e else None for e in ent]

err = [error(i) for i in x]

fig = plt.figure()
# Create 1x1 "subplot" and return the first subplot and return the subplot axes.
ax = plt.subplot(111)

# Plot output from all impurity function for comparison.
for i, lab, ls, c, in zip( [gin, ent, scaled_ent, err],
                           ['Gini', 'Entropy', 'Scaled entropy', 
                               'Misclassification error'],
                           ['--', '-', '-', '-.'], # Line style 
                           ['green', 'black', 'lightgray', 'red', 'cyan'] ) :
    line_plot = ax.plot( x, #input
                         i, #output
                         label = lab,
                         linestyle = ls,
                         lw = 2, #line width
                         color = c )

ax.legend( loc = 'upper center',
           bbox_to_anchor = (0.5, 1.15), # location of the legend
           ncol = 3, # number of columns that the legend has
           fancybox = True,
           shadow = False )

ax.axhline( y = 0.5,
            linewidth = 1,
            color = 'k',
            linestyle = '--' )

ax.axhline( y = 1.0,
            linewidth = 1,
            color = 'k',
            linestyle = '--' )

plt.ylim( [0, 1.1] )
plt.xlabel( 'p(I = 1)' )
plt.ylabel( 'Impurity measure. Higher, the impure.' )
plt.show()

# load the dataset for the practice flower data.
from sklearn import datasets
iris = datasets.load_iris()
X = iris.data[ :, [2, 3] ]
y = iris.target

# Split the data into training and cross-validation test set.
from sklearn.model_selection import train_test_split
X_train, X_val, y_train, y_val = train_test_split( X,
                                                  y,
                                                  test_size = 0.3,
                                                  random_state = 0 )

# Make all features have similar scale.
from sklearn.preprocessing import StandardScaler
sc = StandardScaler()
sc.fit( X_train )
X_train_std = sc.transform( X_train )
X_val_std = sc.transform( X_val )

# Use the Decision Tree from scikit-learn.
from sklearn.tree import DecisionTreeClassifier
tree = DecisionTreeClassifier( criterion = 'entropy',
                               max_depth = 3,
                               random_state = 0 )

# Tree does not need StandardScaler.
tree.fit( X_train, y_train )

# Combine the train and test data so that we can first perform test on the test 
# region and plot both trian and test data.
X_combined = np.vstack( (X_train, X_val) )
y_combined = np.hstack( (y_train, y_val) )

from plotDecisionRegions import plotDecisionRegions
plotDecisionRegions( X_combined,
                     y_combined,
                     classifier = tree,
                     test_idx = range(105,150) )

plt.xlabel( 'petal legnth (cm)' )
plt.ylabel( 'ptel width (cm)' )
plt.legend( loc = 'upper left' )
plt.show()

# Instead of using a signle decision tree, use Random Forest.
from sklearn.ensemble import RandomForestClassifier
forest = RandomForestClassifier( criterion = 'entropy',
                                 n_estimators = 10,
                                 random_state = 1,
                                 n_jobs = 2 )

forest.fit( X_train, y_train )
# Since the X_train was passed by reference, X_combine contains the changed 
# X_train.
plotDecisionRegions( X_combined,
                     y_combined,
                     classifier = forest,
                     test_idx = range(105,150) )

plt.xlabel( 'petal length' )
plt.ylabel( 'petal width' )
plt.legend( loc = 'upper left' )
plt.show()

# Practice K-nearest-neighbor classifier.
from sklearn.neighbors import KNeighborsClassifier
knn = KNeighborsClassifier( n_neighbors = 5,
                            p = 2,
                            metric = 'minkowski')

knn.fit( X_train_std, y_train )

# The combined version now should be the standardized version since KNN is 
# sensitive scaling.
X_combined = np.vstack( (X_train_std, X_val_std) )
y_combined = np.hstack( (y_train, y_val) )

plotDecisionRegions( X_combined,
                     y_combined,
                     classifier = knn,
                     test_idx = range(105,150) )
plt.xlabel( 'petal length (standardized)' )
plt.ylabel( 'petal width (standardized)' )
plt.show()
