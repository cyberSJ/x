# Tests my arbitrary data and arbitrary decision boundary in 2D.
# 2 features.
import numpy as np
XTest = np.array([[2,4], [4,2], [4,4], [5,2], [6,5],   # samples labeled as 0.
                  [2,8], [3,6], [3,10], [4,7], [6,9]]) # samples labled as 1.
print("XTest:\n", XTest)

features = np.array(['featureX', 'featureY']) # features must be 1-D array in order to feed it in to the
                                              # pandas.DataFrame.
print("features:\n", features)

yTest = np.array([0, 0, 0, 0, 0,
                  1, 1, 1, 1, 1])
print("yTest:\n", yTest)
print("Shape of yTest: ", yTest.shape)

import pandas as pd
KnnInput = pd.DataFrame(XTest,
                        columns=features) # columns must be a 1-D array.
print("KnnInput:\n", KnnInput)

# Visualize the input
from matplotlib import cm
pd.plotting.scatter_matrix(KnnInput,
                           c=yTest,
                           marker='o',
                           s=30,
                           alpha=0.8,
                           cmap=cm.RdYlGn)

import matplotlib.pyplot as plt
#plt.show()

from sklearn.neighbors import KNeighborsClassifier
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(XTest,
        yTest)

XNew = np.array([[5,1], [1,8], [7,5], [4,6]]) # Should predict 0 1 0 1.
yTruth = np.array([0, 1, 0, 1])
prediction = knn.predict(XNew)
print("Prediction: ", prediction)
print("Score: {:.2f}".format(knn.score(XNew, yTruth)))
