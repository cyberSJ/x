#!/usr/bin/python

# Check the sklearn version
import sklearn
print('The scikit-learn version is {}.'.format(sklearn.__version__))

# Load the test dataset. See Ch3-1.
from sklearn import datasets
import numpy as np
iris = datasets.load_iris()
X = iris.data[ :, [2, 3] ]
y = iris.target

# Split the dataset in to train and test set. See Ch3-1.
# For now, we are not going to divide the dataset into train, 
# cross-validation, and test set because the Python Machine Learning textbook 
# only udivides the dataset into train and test regions.
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split( X, 
                                                     y, 
                                                     test_size = 0.3,
                                                     random_state = 0 )

# Re-scale the input sample features so that each feature type is relatively 
# comparable to each other.
from sklearn.preprocessing import StandardScaler
sc = StandardScaler()
sc.fit( X_train )
X_train_std = sc.transform( X_train )
X_test_std = sc.transform( X_test )

# Use linear kernel SVM. See Ch3-4.
from sklearn.svm import SVC
from plotDecisionRegions import plotDecisionRegions

# We are going to use a linear kernel, which is as similiar as possible to 
# logistic regression.
svm = SVC( kernel="linear",
           C = 1.0,
           random_state = 0 )

svm.fit( X_train_std,
         y_train )

X_combined_std = np.vstack( (X_train_std, X_test_std) )
y_combined = np.hstack( (y_train, y_test) )

import matplotlib.pyplot as plt

plotDecisionRegions( X_combined_std,
                     y_combined,
                     classifier = svm,
                     test_idx = range(105, 150) )

plt.xlabel( 'petal length [standardized]' )
plt.ylabel( 'petal width [standardized]' )
plt.legend( loc = 'upper left' )
plt.show()
