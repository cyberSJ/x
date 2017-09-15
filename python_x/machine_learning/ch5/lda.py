#!/usr/bin/env python

import numpy as np
import pandas as pd 
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.lda import LDA

df_wine = pd.read_csv(
    'https://archive.ics.uci.edu/ml/machine-learning-databases/wine/wine.data',
    header = None )

################################################################################
# Standardize the data.
################################################################################

# Extract the data
X = df_wine.iloc[:, 1:].values
y = df_wine.iloc[:, 0].values

X_train, X_test, y_train, y_test = train_test_split( X,
                                                     y,
                                                     test_size = 0.3,
                                                     random_state = 0 )

# Standardize the training data.
sc = StandardScaler()
X_train_std = sc.fit_transform( X_train )
X_test_std = sc.transform( X_test )

################################################################################
# Create the mean vector.
################################################################################

# For each label, calculate the mean of the input sample that corresponds to the
# label.
np.set_printoptions( precision = 4 )
mean_vecs = []
for label in range(1,4) :
    mean_vecs.append( np.mean(X_train_std[y_train == label], axis = 0) )
    print( 'mean vector {}: {}\n'.format(label, mean_vecs[label-1]) )

################################################################################
# Calculate the scatter matrix.
################################################################################

num_feat = 13
scatter_matrix = np.zeros( (num_feat, num_feat) )

# For each label, calculate the scatter matrix and sum them up.
for label, mean_vec in zip( range(1,4), mean_vecs ) :
    class_scatter = np.zeros( (num_feat, num_feat) )
    mean_vec.reshape( num_feat , 1 )

    for row in X_train[ y_train == label ] :
        row = row.reshape( num_feat, 1 )
        class_scatter += (row - mean_vec).dot((row - mean_vec).T)

    # sung why are we not directly adding to the scatter_matrix in the second
    # for-loop?
    scatter_matrix += class_scatter

print( 'Within-class scatter matrix: {}{}'.format(s_w.shape[0], s_w.shape[1]) )

# Print the class label distribution count
print( 'Class label distribution count: {}'.format(np.bincount(y_train)[1:]) )
# sung What if I remove the [1:]?
print( 'Class label distribution count: {}'.format(np.bincount(y_train)) )

################################################################################
# Calculate the scaled scatter matrix using covariance function.
################################################################################

# We are calculating a scatter matrix because LDA requires it. We scale the
# matrix because LDA assumes uniform class label distribution. By scaling, it is
# same thing is normalizing the number of class label distribution.
scaled_scatter_matrix = np.zeros( (num_feat, num_feat) )

# y_train == label gives indicies which the elements match the label.
# X_train_std step gives N x M matrix for input samples. N: number of samples
# that match the number of elements that match the lable. M: number of features
# in the high-dimensional feature space..
# Transposing it give M x N matrix.
# np.cov() gives M by M vector that is the scaled scatter matrix.
# The final total scaled scatter matrix is M by M matrix.
for label in range(1,4) :
    scaled_scatter_matrix += np.cov( X_train_std[ y_train == label ].T )

print( 'Scaled within-class scatter matrix: {} x {}'.format(
            scaled_scatter_matrix.shape[0],
            scaled_scatter_matrix.shape[1]) )

################################################################################
# Calculate the between-class scatter matrix.
################################################################################

# Calculate the overall sum. All input samples are contained in the X_train_std.
# Summing over the first axis (row) means all rows will collapse in the N X M
# matrix, giving 1 x M vector.
mean_overall = np.sum( X_train_std, axis = 0 )

# Select one mean vector 1 x M. M is the number of features.
# Subtract with the overall mean.
# Get the number of samples that belong to that label by getting the row number
# of the matrix that is extracted with the matching class label.
# Multiply with the number of samples that belong to that label.
for idx, vec in enumerate( mean_vecs ) :

    # idx starts with 0 where as the y_train is in [1,3].
    num_samples = X_train_std[ y_train == idx + 1].shape[0]

    # mean_diff is 1 x M vector.
    mean_diff = vec - mean_overall

    # mean_diff is now M x 1 vector ready for multiplicatiion that produces
    # matrix.
    mean_diff.reshape(num_feat, 1 )

    # sung Can't I use * operator for the list in Python?
    s_b += num_samples * mean_diff.dot(mean_diff.T)
    
print( 'Between-class scatter matrix: {} x {}'.format(
            s_b.shape[0],
            s_b.shape[1]) )

################################################################################
# Obtain the eigen pairs from the multiplied scatter matrices.
################################################################################

# Calculate the inverse(s_w) * s_b, which is the final combined scatter
# matrices. Then get the eigens.
eigen_val, eigen_vec = np.linalg.eig( 
                        np.linalg.inv(scaled_scatter_matrix).dot(s_b) )

# Create eigen value & vector pairs.
# Python list does not have be in the same type or dimension. The are like 
# cells. Use the for loop to insert each of the entries in the eigen val/vec
# structure into the eigen_pair structure. The eigen vector is the M x M matrix
# where M is the number of features in the input data.
eigen_pairs = [(np.abs(eigen_val[i]), egien_vec[:, i])  \
               for i in range(len(eigen_val))]

# Sort the eigens from the highest to the lowest eigen value. Use the first
# field (which is the eigen value, as oppposed to the eigen vector which is the
# second field) as the key.
eigen_pairs = sorted( eigen_pairs, 
                      key=lambda k: k[0], # lamda syntax: lambda arg: body
                      reverse = True )

print( 'Eigenvalues in decreasing order:\n' )
for pair in eigen_pairs:
    print( pair[0] )

################################################################################
# Plot the discriminability ratio.
################################################################################

# Obtain the percentage of how much each eigen value occupies to the total.
sum_eig_vals = sum(eigen_val.real)
ratios = [ (i/sum_eig_vals) for i in sorted(eigne_val.real, reverse = True) ]

# Calculate the cumulative sum after each eigen value.
cum_sum = np.cumsum( ratios )

# Bar graph.
plt.bar( range(1, 14),
         ratios,
         alpha = 0.5,
         align = 'center',
         label = 'individual discriminability' )

# Cumulative step graph.
plt.step( range(1, 14),
          cum_sum,
          where = 'mid',
          label = 'cumulative discriminability' )

plt.ylabel( 'Discriminability ratio' )
plt.xlabel( 'eigen vectors. i.e. the linear discriminants' )
plt.ylim( [-0.1, 1.1] )
plt.legend( loc='best' )
plt.show()

################################################################################
# Create the down-conversion matrix
################################################################################

# np.hstack() takes a list. This list contains two sets M x 1 vectors. The first
# index selectcs which pair, the second index selects the second element in the
# pair, which is the eigen vector. The next square brackets reshapese the vector
# so that it becoms the M x 1 vector. It pushes all elements in to the first
# dimension, and the second dimension becomes 1 due to np.newaxis.
w = np.hstack( (eigen_pairs[0][1][:, np.newaxis].real,
                eigne_pairs[1][1][:, np.newaxis].real) )

print( 'Matrix w:\n', w )

################################################################################
# Project the input data to lower dimension
################################################################################

# The input is the X_train_std. It is N x M matrix.  w is M x 2 matrix, where M 
# is the #{features}. The result is N x 2 matrix, where N is the #{samples}.
X_train_lda = X_train_std.dot(w)

# Scatter plot the input and color them according to the label.
colors = [ 'r', 'b', 'g' ]
markers = [ 's', 'x', 'o' ]

# sung What is the -1 multiplication for?
for label, color, marker in zip( np.unique(y_train), colors, markers ) :
    plt.scatter( X_train_lda[ y_train == label, 0] * -1,
                 X_train_lda[ y_train == label, 1] * -1,
                 label = label,
                 c = color,
                 marker = marker )

plt.xlabel( 'LD 1' )
plt.ylabel( 'LD 2' )
plt.legend( loc = 'lower right' )
plt.show()

################################################################################
# Use LDA library from sklearn.
################################################################################
lda = LDA( n_components = 2 )
X_train_lda = lda.fit_transform( X_train_std,
                                 y_train )

# Train the logistic regression algorithm using the down-converted input.
lr = LogisticRegression()
lr = lr.fit( X_train_lda,
             y_train )
plotDecisionRegions( X_train_lda,
                     y_train,
                     classifier = lr )
plt.xlabel( 'LD 1' )
plt.ylabel( 'LD 2' )
plt.legend( loc = 'lower left' )
plt.show()

# Test the trained logistic regression algorithm

X_test_lda = lda.transform( X_test_std )
plotDecisionRegions( X_test_lda,
                     y_test,
                     classifer = lr )
plt.xlabel( 'LD 1' )
plt.ylabel( 'LD 2' )
plt.legend( loc = 'lower left' )
plt.show()
