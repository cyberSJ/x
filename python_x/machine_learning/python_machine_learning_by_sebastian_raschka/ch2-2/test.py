#!/usr/bin/python
# Tests the Perceptron machine learning algorithm, which is one of the most
# basic machine learning algorithms.
# Usage: ./test.py

# Syntax: from <.py name> import <class name>
from Perceptron import Perceptron
from plotDecisionRegions import plotDecisionRegions
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv(
        'https://archive.ics.uci.edu/ml/machine-learning-databases/iris/iris.data',
        header = None )

print df.tail()

# Classification label
y = df.iloc[0:100, 4].values
y = np.where( y == 'Iris-setosa', -1, 1 )

# Input matrices. 100 samples, each with 2 features.
X = df.iloc[0:100, [0,2]].values

# Plot Setosa in red
plt.scatter( X[:50, 0],  # feature 1
             X[:50, 1],  # feature 2
             color = 'red',
             marker = 'o',
             label = 'setosa' )

# Plot Versicolor in blue
plt.scatter( X[50:, 0],
             X[50:, 1],
             color = 'blue',
             marker = 'x',
             label = 'versicolor' )

plt.xlabel( 'petal length' )
plt.ylabel( 'sepal length' )
plt.legend( loc='upper left' )
plt.show()

# Learn the classification
perceptron = Perceptron( 0.1, 10 )
perceptron.fit( X, y )
plt.plot( range(1, len(perceptron.errors_) + 1),
          perceptron.errors_,
          marker = 'o' )
plt.xlabel( 'number of iteration' )
plt.ylabel( 'Number of misclassification in each iteration' )
plt.show()

plotDecisionRegions( X, y, classifier = perceptron )
plt.xlabel( 'sepal length (cm)' )
plt.ylabel( 'petal length (cm)' )
plt.legend( loc = 'upper left' )
plt.show()
