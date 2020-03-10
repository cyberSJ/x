#!/usr/bin/env python
# Ch4. Preprocessing.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import sbs

# Download the CSV data for the Wine data.
# df stands for DataFrame structure defined by panda.
df_wine = pd.read_csv( 
    'https://archive.ics.uci.edu/ml/machine-learning-databases/wine/wine.data',
    header = None )

# Manually define the column names.
df_wine.columns = [ 'Class label',
                    'Alcohol',
                    'Malic acid',
                    'Ash',
                    'Alcalinity of ash',
                    'Magnesium',
                    'Total phenols',
                    'Flavanoids',
                    'NonFlavanoid phenols',
                    'Proanthocyanins',
                    'Color intensity',
                    'Hue',
                    'OD280/OD315 of diluted wines',
                    'Proline' ]

# Separate the data into test and train set. And get the numpy from the
# DataFrame structure.
X = df_wine.iloc[:,1:].values
y = df_wine.iloc[:,0].values
X_train, X_test, y_train, y_test = train_test_split( X,
                                                     y,
                                                     test_size = 0.3,
                                                     random_state = 0 )

# Scale the raw data using the mean and the standard deviation.
stdsc = StandardScaler()
X_train_std = stdsc.fit_transform( X_train )
X_test_std = stdsc.transform( X_test )

# Instantiate a figure to plot the graph.
fig = plt.figure()

# First 1 means row of the plot window.
# Second 1 means the column of the plot window.
# Third 1 means select the 1st subplot.
ax = plt.subplot(111)

colors = [ 'blue',
           'green',
           'red',
           'cyan',
           'magenta',
           'yellow',
           'black',
           'pink',
           'lightgreen',
           'lightblue',
           'gray',
           'indigo',
           'orange' ]

weights = []
params = []

for c in np.arange(-4, 6) :
    lr = LogisticRegression( penalty = 'l1', # Use L1 regularization, which
                                             # produces more sparse matrix 
                                             # compared to L2 regularization.
                             C = 10**c,
                             random_state = 0 )
    lr.fit( X_train_std,
            y_train )

    # Append the 13 coefficients for the class 1 (the second class).
    # There are total of 3 lables (classes) in the data.
    weights.append( lr.coef_[1] )

    params.append(10**c)

# Convert the Python array to Numpy array so that we can plot.
weights = np.array( weights )

for feature, color in zip( range(weights.shape[1]), colors) :
    plt.plot( params, # x-value is the regularization strength
              weights[:, feature],
              label = df_wine.columns[feature + 1], # First column is the label.
              color = color )

plt.axhline( 0, 
             color = 'black', 
             linestyle = '--',
             linewidth = 3 )

plt.xlim( [10**(-5), 10**5] )
plt.ylabel( 'weight coefficients value for each feature' )
plt.xlabel( 'Inverse of Regularization strength' )
plt.xscale( 'log' )
plt.legend( loc = 'upper left' )

ax.legend( loc = 'upper center',
           bbox_to_anchor = (1.38, 1.03),
           ncol = 1, 
           fancybox = True )

plt.show()

from sklearn.neighbors import KNeighborsClassifier
knn = KNeighborsClassifier( n_neighbors = 2 )

# Create SBS. Perform reduction until there is one feature left.
sbs = sbs.SBS( knn,
               k_features = 1 )

# Actually perform the feature reduction.
sbs.fit( X_train_std, y_train )

# Now that we have performed the feature reduction and gathered the scores, we
# can plot #{feature} vs. score.
array_num_features = [len(k) for k in sbs.subsets_]
plt.plot( array_num_features, 
          sbs.scores_, 
          marker ='o' )
plt.ylim( [0.7, 1.1] ) # [y_min, y_max]
plt.ylabel( 'Accuracy percentage' )
plt.xlabel( 'Number of features' ) # sung Shouldn't this be number of feature reductions?
plt.grid()
plt.show()

# Get the feature selection from the 9th feature-reducing iteration.
k5_index = list(sbs.subsets_[8])    # sung experiement not using list().
print('k5_index is a list []: ')
print(k5_index)
print('without is a tuple (): ')
print(sbs.subsets_[8])

# First column is the actual label of the wine, so discard that label. After
# that we are left with the full features, so only select the indexed feature.
# This was we can read what features were the essential features and what
# features were irrelevant and therefore sometimes hurt our KNN algorithm.
print( df_wine.columns[1:][k5_index] )

# Compare with the case when we did not perform feature reduction.
knn.fit( X_train_std, y_train )

# We can also produce score on the trainig set as well.
print( 'Training accuracy: ', knn.score(X_train_std, y_train) )
 
# To product the data, predict on the new data. KNN has the score() function
# that can perform the predictition and comparison with the true label all at
# once.
print( 'Test accuracy: ', knn.score(X_test_std, y_test) )

# Print the KNN trained with only the selected, effective features. Observe that
# the KNN trained with the selected useful features gave higher prediction rate.
# This is because the irrelevant, non-effective features gives the learning
# algorithm a hurting factor because the measure of "nearness" in the KNN
# algorithm is hindered by the irrelevant features that are "far" in the feature
# space.
knn.fit( X_train_std[:, k5_index], y_train )
print( 'Training accuracy: ', knn.score(X_train_std[:, k5_index], y_train) )
print( 'Test accuracy: ', knn.score(X_test_std[:, k5_index], y_test) )

# We can even lay out the features in the order of importance! This requires
# using Random Forest to figure out which feature gave the most informational
# gain in multiclassification.
from sklearn.ensemble import RandomForestClassifier

# The RandomForestClassifier already has capability to store the importances, so
# train the RandomForest classifier.
forest = RandomForestClassifier( n_estimators = 10000, # many estimators
                                 random_state = 0,
                                 n_jobs = 1)
forest.fit( X_train,
            y_train )

# Extract the importances of all features. This is the vector with #{features}
# elements. The value is the importance measure itself.
importances = forest.feature_importances_

# Sort the importance from high to low. [::-1] follows the syntax of 
# [<stop>:<end>:<step>], so [::-1] means "reverse" because the step is -1.
indices = np.argsort(importances)[::-1]

# Now plot the importance with each category. First the x-axis is just a number
# 0 through the #{features}. The y-axis is the importance value.
plt.bar( range(X_train.shape[1]),
         importances[indices],
         color = 'lightblue',
         align = 'center' )

# Now label the x-axis ticks.
feature_labels = df_wine.columns[1:]
plt.xticks( range(X_train.shape[1]),
            feature_labels[indices],
            rotation = 90 )
plt.title( 'Feature Importances' )
plt.xlim( [-1, X_train.shape[1]] )

# Auto-fit the graph to the plot window.
plt.tight_layout()
plt.show()

# Print out the label and the importance manually, too. For each of the feature
# print the statistics.
for feature in range(X_train.shape[1]):
    # %2d: Number up to 2 decimal. Used to print the order number.
    # %-*s: Arbitrary string. Used for the feature lable string.
    # %f: Floating number. Used for the importance values.
    print( "%2d) %-*s %f" % (feature + 1, 
                             30, 
                             feature_labels[indices[feature]],
                             importances[indices[feature]]) )
