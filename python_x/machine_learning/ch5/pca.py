#!/usr/bin/env python
# Conclusion: Eigen values mean how "main" that eigen vector is in determining
# the spread of the data. Higher eigne value means the principa component (eigen
# vector) is more important.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler


df_wine = pd.read_csv(
    'https://archive.ics.uci.edu/ml/machine-learning-databases/wine/wine.data',
    header = None)

# Select all rows, 2nd-end columns because the 1st column is the label of the
# wine. values returns the Python lists I think.
X = df_wine.iloc[:,1:].values
y = df_wine.iloc[:,0].values

# Divide the input sample into test and training set.
X_test, X_train, y_test, y_train = train_test_split( X,
                                                     y,
                                                     test_size = 0.3,
                                                     random_state = 0 )
########################
# Begin performing PCA. 
########################

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

###############################
# Plot the explained variances 
###############################

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
