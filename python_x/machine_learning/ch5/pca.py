#!/usr/bin/env python
# Conclusion: Eigen values mean how "main" that eigen vector is in determining
# the spread of the data. Higher eigne value means the principa component (eigen
# vector) is more important.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from plotDecisionRegions import plotDecisionRegions
from sklearn.decomposition import PCA
from sklearn.linear_model import LogisticRegressoin
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

df_wine = pd.read_csv(
    'https://archive.ics.uci.edu/ml/machine-learning-databases/wine/wine.data',
    header = None )

# Select all rows, 2nd-end columns because the 1st column is the label of the
# wine. values returns the Python lists I think.
X = df_wine.iloc[:,1:].values
y = df_wine.iloc[:,0].values

# Divide the input sample into test and training set.
X_test, X_train, y_test, y_train = train_test_split( X,
                                                     y,
                                                     test_size = 0.3,
                                                     random_state = 0 )

################################################################################
# Begin performing PCA. 
################################################################################

# Standardize the input
sc = StandardScaler()
X_train_std = sc.fit_transform( X_train )
X_test_std = sc.transform( X_test )

# First, compute the covariance matrix.
cov_mat = np.cov(X_train_std.T)

# Coursera tells me to use SVD, but Safari tells me to manually use the 
# linalg.eig function to calculatethe eigenvalue.
eig_val, eig_vec = np.linalg.eigh( cov_mat )
print( 'Eigen values: \n{}'.format(sorted(eig_val, reverse = True)) )

################################################################################
# Plot the explained variances.
################################################################################

# each eigen value over the sum of all eigen values.
total = sum( eig_val )
explained_variances = [ (i / total) for i in sorted(eig_val, reverse = True) ]

# Use the cumulative summation function to incrementally calculate the
# cumulative sum for each of the members in the array.
cum_exp_var = np.cumsum( explained_variances )
plt.bar( range(1,14), 
         explained_variances,
         alpha = 0.5,
         align = 'center',
         label = 'individual explained variances' )
plt.step( range(1,14),
          cum_exp_var,
          where = 'mid',
          label = 'cumulative explained variances' )
plt.xlabel( 'eigen values (principal compoenents)' )
plt.ylabel( 'explained variances' )
plt.legend( loc = 'best' )
plt.show()

################################################################################
# Down-convert to the lower dimension.
################################################################################

# Sort the eigen vector according to the engien value. Using parenthesis, we can
# pair the one eigen value, and one eigen vector that contains multiple values.
eig_pairs = [(np.abs(eig_val[i]), eig_vec[:, i]) for i in range(len(eig_val))]
eig_pairs.sort(reverse=True)

# Select the top 2 eigen vector, which is responsible for the 57% of the
# variance in the data according to the eigen value graph. Wit the top eigen
# vector, create the w matrix that will be used to down-covert the high
# dimensional data into a lower dimensional data.
# np.newaxis is used to make the eigenvector into 13x1 vector I think.. Not sure
# exactly what np.newaxis does in this context but that's what I understand so
# far..
w = np.hstack( (eig_pairs[0][1][:, np.newaxis],
                eig_pairs[1][1][:, np.newaxis]) ) 
print( 'Matrix w:\n', w )

# Multiply the original high-dimension data with the w matrix to down-convert. 
X_train_pca = X_train_std.dot(w)

################################################################################
# Visualize the down-coverted dimension.
################################################################################
colors = [ 'r', 'b', 'g' ] # red, blue, green
markers = [ 's', 'x', 'o' ] # square, x, o

# Extract the unique labels in the data label set and use different colors and
# markers for each type of the labels. For each data set that resulted in that
# type of label, plot the data with the color and marker.
for lab, color, marker in zip( np.unique(y_train), colors, markers ):
    plt.scatter( X_train_pca[ y_train == lab, 0],
                 X_train_pca[ y_train == lab, 1],
                 label = lab,
                 c = color,
                 marker = m )

plt.xlabel( 'PC 1' )
plt.ylabel( 'PC 2' )
plt.legend( loc = 'lower left' )
plt.show()

################################################################################
# Use PCA library to compare the PCA implemented by hand.
################################################################################

# n_components is the final dimension of the input samples we want to
# down-convert to.
pca = PCA( n_components = 2 )
lr = LogisticRegression()
X_train_pca = pca.fit_transform( X_train_std )
X_test_pca = pca.transform( X_test_std )

# Using the PCA'ed input data, train the logistic regression algorithm.
lr.fit( X_train_pca, y_train )
plotDecisionRegions( X_train_pca, 
                     y_train,
                     classifier = lr )
plt.xlabel( 'PC1' )
plt.ylabel( 'PC2' )
plt.legend( loc = 'lower left' )
plt.show()

# Plot the test data 
plotDecisionRegions( X_test_pca,
                     y_test,
                     classiferi =lr )
plt.xlabel( 'PC1' )
plt.ylabel( 'PC2' )
plt.legend( loc = 'lower left' )
plt.show()

################################################################################
# For fun, view the explained variance ratios using the PCA library instead of
# trying to prepare the ratios by hand.
################################################################################

# n_components = None means we are going to use all high-dimensional data.
pca = PCA( n_components = None )

# Fit the data using PCA.
X_train_pca = pca.fit_transform( X_train_std )

# Now the PCA algorithm has processed the data, obtain the explained variance
# ratios.
print( pca.explained_variances_ratio_ )
