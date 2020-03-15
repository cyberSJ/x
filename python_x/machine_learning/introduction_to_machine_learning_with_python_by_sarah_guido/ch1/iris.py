from sklearn.datasets import load_iris
import sklearn
print("sklearn version: ", sklearn.__version__)
irisDataset = load_iris()
print("Keys of iris dataset:\n", irisDataset.keys())
print(irisDataset['DESCR'][:250]) # First 250 characters of the description of the iris dataset.
print("Target names: ", irisDataset['target_names'])
print("Feature names: ", irisDataset['feature_names'])
print("Type of data: ", type(irisDataset['data']))
print("Shape of data: ", irisDataset['data'].shape)
print("First five samples: \n", irisDataset['data'][:5])
print("Type of target: ", type(irisDataset['target']))
print("Shape of target: ", irisDataset['target'].shape)
print("Target: \n", irisDataset['target'])

from sklearn.model_selection import train_test_split
XTrain, XTest, yTrain, yTest = train_test_split(irisDataset['data'], 
                                                irisDataset['target'], 
                                                random_state=0,
                                                train_size=0.02)
print("XTrain shape: ", XTrain.shape)
print("yTrain shape: ", yTrain.shape)
print("XTest shape: ", XTest.shape)
print("yTest shape: ", yTest.shape)

import pandas as pd

# Pandas plotting tool requires the input to be in DataFrame.
irisDataFrame = pd.DataFrame(XTrain, columns=irisDataset.feature_names)
print("Type of feature_names: ", type(irisDataset.feature_names))

import mglearn

# Show all possible graphs of 2-feature comparison. This helps visualize the input data.
pd.plotting.scatter_matrix(irisDataFrame, # The input values.
                           c=yTrain, # The labels for each input value. Used for coloring each data point according
                                     # to its class.
                           figsize=(15, 15),
                           marker='o',
                           hist_kwds={'bins': 20},
                           s=60, # size of the marker
                           alpha=0.8,
                           cmap=mglearn.cm3)

# Actually show the graph.
import matplotlib.pyplot as plt
#plt.show()

# from <module> import <class>
from sklearn.neighbors import KNeighborsClassifiej
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(XTrain, yTrain)

import numpy as np
XNew = np.array([[5, 29, 1, 0.2]])
print("XNew shape: ", XNew.shape)

prediction = knn.predict(XNew)
print("Prediction: ", prediction)
print("Predicted target name: ", irisDataset['target_names'][prediction])

yPredictTest = knn.predict(XTest)
print("Test set predictions: ", yPredictTest)
print("Test set score: {:.2f}".format(np.mean(yPredictTest == yTest)))
print("Test set score using score(): {:.2f}".format(knn.score(XTest, yTest)))
