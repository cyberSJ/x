#!/usr/bin/env python
# Kernel Principal Component Analysis.

import matplotlib.pyplot as plt
import numpy as np
from scipy import exp
from scipy.linalg import eigh
from scipy.spatial.distance import pdist, squareform

def kpca( X, 
          gamma, 
          n_components ):

    # The Kernel PCA needs the euclidean distance squared. Use SciPy function to
    # do that. The result is N^2 x 1 vector where N is #{samples}.
    sq_dists = pdist( X,
                      'sqeuclidean' )

    # Reshape the result in to the N x N matrix where N is #{samples}.
    mat_sq_dists = squareform( sq_dists )

    # Finish the kernelization with the exponent operation. I think this step
    # should have been done earlier but I think matrix operation is better than
    # vector operation in this situation? No not really. This is where the
    # lower-dimensional data is directly transferred to high dimensional data.
    K = exp(-gamma * mat_sq_dists )

    # Center the kernel matrix with pure Python math. What centering the kernel
    # means, and why doing this kind of math achieves centering are still left
    # as questions for me.
    N = K.shape[0]
    one_over_n = np.ones( (N, N) ) / N
    K = K - one_over_n.dot( K ) - K.dot( one_over_n ) + \
        one_over_n.dot( K ).dot( one_over_n )

    # Obtain the eigen  pairs from the centered kernel matrix using numpy. The
    # eigen vecs are in increasing order.
    eig_vals, eig_vecs = eigh( K )

    # Collect the top k eigenvectors (which are samples projected onto the
    # higher dimensional space). Get the last k columns. Accessing eig_vec's one
    # column gives a vertical vector, so using Numpy's column_stack() makes the
    # resulting matrix an D x K, where D is #{original features} and K is the
    # #{projected features}.
    alpha = np.column_stack( 
        (eig_vecs[:, -i] for i in range(1, n_components + 1)) )

    # Collect the top eigen values.
    lambdas = [ eig_vals[-i] for i in range(1, n_components + 1) ]

    return alpha, lambdas

def project( x_new, X, gamma, alphas, lambdas ):
    # For each sample in X (each row), get the squared distance between the new
    # sample and each sample in X. Get the total of those distances.
    pair_dist = np.array( [np.sum((x_new - row)**2) for row in X] )

    # Finish up the kernel trick formula.
    k = np.exp( -gamma * pair_dist )

    return k.dot( alphas / lambdas )

################################################################################
# Create a test data set that is non-linearly separable.
################################################################################

from sklearn.datasets import make_moons
X, y = make_moons( n_samples = 100,
                   random_state = 123 )

# First arg: y == 0 gives indicies to the samples that has label 0. We want the 
# first feature value so one more 0.
# Second arg: We want the second feature value so 1 for the second arg.
plt.scatter( X[ y == 0, 0 ],
             X[ y == 0, 1 ],
             color = 'red',
             marker = '^',
             alpha = 0.5 )

plt.scatter( X[ y == 1, 0 ],
             X[ y == 1, 1 ],
             color = 'blue',
             marker = 'o',
             alpha = 0.5 )

plt.show()

################################################################################
# Try to see how the linear PCA cannot achieve separability on this data
################################################################################

from sklearn.decomposition import PCA
scikit_pca = PCA( n_components = 2 )

X_spca = scikit_pca.fit_transform( X )

# fig is unused.
fig, ax = plt.subplots( nrows = 1,
                        ncols = 2,
                        figsize = (7, 3) )

ax[0].scatter( X_spca[y == 0, 0],
               X_spca[y == 0, 1],
               color = 'red',
               marker = '^',
               alpha = 0.5 )

ax[0].scatter( X_spca[y == 1, 0],
               X_spca[y == 1, 1],
               color = 'blue',
               marker = 'o',
               alpha = 0.5 )
ax[0].set_xlabel( 'PC 1' )
ax[0].set_ylabel( 'PC 2' )

# Ignore the second dimension and collapse down to the first dimension to see
# that linear separability is not possible.
ax[1].scatter( X_spca[y==0, 0],
              np.zeros((50,1)) + 0.02,
              color = 'red',
              marker = '^',
              alpha = 0.5 )

ax[1].scatter( X_spca[y==1, 0],
               np.zeros((50,1)) - 0.02,
               color = 'blue',
               marker = 'o',
               alpha = 0.5 )

ax[1].set_ylim( [-1 ,1] )
ax[1].set_yticks( [] )
ax[1].set_xlabel( 'PC 1' )
plt.show()

################################################################################
# Show the power of the kernel PCA in dividing the non-linearly separable case. 
################################################################################
from matplotlib.ticker import FormatStrFormatter
alphas, lambdas = kpca( X,
                        gamma = 15,
                        n_components = 1 )

x_new = X[25]
print( x_new )
x_proj = alphas[25]
print( x_proj )

x_reproj = project( x_new,
                    X,
                    gamma = 15,
                    alphas = alphas,
                    lambdas = lambdas )
print( x_reproj )

plt.scatter( alphas[ y == 0, 0],
             np.zeros( (50) ),
             color = 'red',
             marker = '^',
             alpha = 0.5 )

plt.scatter( alphas[ y == 1, 0],
             np.zeros( (50) ),
             color = 'blue',
             marker = 'o',
             alpha = 0.5 )

plt.scatter( x_proj, 
             0, 
             color = 'black', 
             label = 'original projection of point X[25]',
             marker = '^',
             s = 100 )

plt.scatter( x_reproj,
             0,
             color = 'green',
             label = 'rempapped point X[25]',
             marker = 'x',
             s = 500  ) # scale I think.

plt.legend( scatterpoints = 1 )
plt.show()

#fig, ax = plt.subplots( nrows = 1,
#                        ncols = 2,
#                        figsize = (7, 3) )
#
#ax[0].scatter( alphas[y == 0, 0],
#               alphas[y == 0, 1],
#               color = 'red',
#               marker = '^',
#               alpha = 0.5 )
#
#ax[0].scatter( alphas[y == 1, 0],
#               alphas[y == 1, 1],
#               color = 'blue',
#               marker = 'o',
#               alpha = 0.5 )
#
#ax[0].set_xlabel( 'PC 1' )
#ax[0].set_ylabel( 'PC 2' )
#ax[0].xaxis.set_major_formatter( FormatStrFormatter( '%0.1f' ) )
#
#ax[1].scatter( alphas[y == 0, 0],
#               np.zeros( (50, 1) ) + 0.02,
#               color = 'red',
#               marker = 'o',
#               alpha = 0.5 )
#
#ax[1].scatter( alphas[y == 1, 0],
#               np.zeros( (50, 1) ) - 0.02,
#               color = 'blue',
#               marker = 'o',
#               alpha = 0.5 )
#
#ax[1].set_xlabel( 'PC 1' )
#ax[1].set_ylim( [-1, 1] )
#ax[1].set_yticks( [] )
#ax[1].xaxis.set_major_formatter( FormatStrFormatter('%0.1f') )
#plt.show()

################################################################################
# Use KernelPCA class in scikit-learn.
################################################################################
from sklearn.decomposition import KernelPCA
X, y = make_moons( n_samples = 100,
                   random_state = 123 )

scikit_kpca = KernelPCA( n_components = 2,
                         kernel = 'rbf',
                         gamma = 15 )

X_scikit_kpca = scikit_kpca.fit_transform( X )

plt.scatter( X_scikit_kpca[ y == 0, 0 ],
             X_scikit_kpca[ y == 0, 1 ],
             color = 'red',
             marker= '^',
             alpha = 0.5 )

plt.scatter( X_scikit_kpca[ y == 1, 0 ],
             X_scikit_kpca[ y == 1, 1 ],
             color = 'blue',
             marker= 'o',
             alpha = 0.5 )
plt.xlabel( 'PC 1' )
plt.ylabel( 'PC 2' )
plt.show()
