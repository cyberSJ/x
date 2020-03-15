# More real-life-like problem.
# Distinguishing kiyeouk, nieun, digeud Korean letters.
# Three samples for each letter.
# Each sample is 5x5 pixels. Row-major (Cover all rows first before going to the next column) since the numpy ndarray is
# row major.
import numpy as np
someArray = np.array([0, 1, 2, 3, 4, 5,])
print("somArray[0:2]: ", someArray[0:2])
print("somArray[2:3]: ", someArray[2:3])
print("somArray[4:6]: ", someArray[4:6])
print("somArray[4:7]: ", someArray[4:7])
print("somArray[-1:7]: ", someArray[-1:7])
print("somArray[-1:8]: ", someArray[-1:8])
print("somArray[-1:6]: ", someArray[-1:6])
print("somArray[-2:6]: ", someArray[-2:6])
print("somArray[1]: ", someArray[1])

from scipy.sparse import csr_matrix
accumulativeCountOfElementPerRow = np.array([0, 0, 3, 4, 5, 5])
columnIndexForEachElement = np.array([1,2,3,3,3])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

kiyeouk0 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                       shape=letterImageSizePixels)

print("kiyeouk0:\n", kiyeouk0.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,2,4,5,6])
columnIndexForEachElement = np.array([2,3,1,3,3,2])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

kiyeouk1 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                       shape=letterImageSizePixels)

print("kiyeouk1:\n", kiyeouk1.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,4,5,6,7])
columnIndexForEachElement = np.array([0,1,2,3,2,2,1])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

kiyeouk2 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                       shape=letterImageSizePixels)

print("kiyeouk2:\n", kiyeouk2.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,1,2,5,5])
columnIndexForEachElement = np.array([1,1,1,2,3])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

nieun0 = csr_matrix((valueForEachElement,
                         columnIndexForEachElement,
                         accumulativeCountOfElementPerRow),
                     shape=letterImageSizePixels)

print("nieun0:\n", nieun0.toarray())

accumulativeCountOfElementPerRow = np.array([0,1,2,3,7,7])
columnIndexForEachElement = np.array([1,1,1,1,2,3,4])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

nieun1 = csr_matrix((valueForEachElement,
                         columnIndexForEachElement,
                         accumulativeCountOfElementPerRow),
                     shape=letterImageSizePixels)

print("nieun1:\n", nieun1.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,1,2,7,7])
columnIndexForEachElement = np.array([0,0,0,1,2,3,4])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

nieun2 = csr_matrix((valueForEachElement,
                         columnIndexForEachElement,
                         accumulativeCountOfElementPerRow),
                     shape=letterImageSizePixels)

print("nieun2:\n", nieun2.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,3,4,7,7])
columnIndexForEachElement = np.array([1,2,3,1,1,2,3])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

digeud0 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                      shape=letterImageSizePixels)

print("digeud0:\n", digeud0.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,3,4,5,8])
columnIndexForEachElement = np.array([1,2,3,1,1,1,2,3])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

digeud1 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                      shape=letterImageSizePixels)

print("digeud1:\n", digeud1.toarray())

accumulativeCountOfElementPerRow = np.array([0,0,3,4,6,8])
columnIndexForEachElement = np.array([1,2,3,1,1,3,1,2])
valueForEachElement = np.ones(len(columnIndexForEachElement))
letterImageSizePixels = (5,5)

digeud2 = csr_matrix((valueForEachElement,
                          columnIndexForEachElement,
                          accumulativeCountOfElementPerRow),
                      shape=letterImageSizePixels)

print("digeud2:\n", digeud2.toarray())

import scipy.sparse as sp
# Reshape the raw input to a sample-feature structure.
kiyeouk0 = kiyeouk0.reshape((1,-1))
kiyeouk1 = kiyeouk1.reshape((1,-1))
kiyeouk2 = kiyeouk2.reshape((1,-1))
nieun0 = nieun0.reshape((1,-1))
nieun1 = nieun1.reshape((1,-1))
nieun2 = nieun2.reshape((1,-1))
digeud0 = digeud0.reshape((1,-1))
digeud1 = digeud1.reshape((1,-1))
digeud2 = digeud2.reshape((1,-1))
XTest = sp.vstack([kiyeouk0,
                   kiyeouk1,
                   kiyeouk2,
                   nieun0,
                   nieun1,
                   nieun2,
                   digeud0,
                   digeud1,
                   digeud2])

# Prepare the input as DataFrame.
import pandas as pd
KnnInput = pd.DataFrame.sparse.from_spmatrix(XTest)
print("KnnInput:\n", KnnInput)

# Prepare the labels.
yTest = np.array([0, 0, 0, 1, 1, 1, 2, 2, 2])

# Visualize the data. But because there are many features, plotting this is very computationally demanding.
#from matplotlib import cm
#pd.plotting.scatter_matrix(KnnInput,
#                           c=yTest,
#                           marker='o',
#                           size=20,
#                           cmap=cm.RdYlGn)

# Train the KNN.
from sklearn.neighbors import KNeighborsClassifier
knn = KNeighborsClassifier(n_neighbors=1)
knn.fit(XTest, yTest)

# Test new kiyouk
accumulative = np.array([0,0,3,4,5,6])
columnIdx = np.array([1,2,3,3,2,1])
values = np.ones(len(columnIdx))
kiyeoukTest0 = csr_matrix((values,
                               columnIdx,
                               accumulative),
                          shape=letterImageSizePixels)
print("kiyeoukTest0:\n", kiyeoukTest0.toarray())
kiyeoukTest0 = kiyeoukTest0.reshape((1,-1))
kiyeoukTest0Pred = knn.predict(kiyeoukTest0)
print("kiyeoukTest0Pred: ", kiyeoukTest0Pred)

# Test new nieun 
accumulative = np.array([0,0,1,2,4,4])
columnIdx = np.array([1,2,3,4])
values = np.ones(len(columnIdx))
nieunTest0 = csr_matrix((values,
                             columnIdx,
                             accumulative),
                         shape=letterImageSizePixels)
print("nieunTest0:\n", nieunTest0.toarray())
nieunTest0 = nieunTest0.reshape((1,-1))
nieunTest0Pred = knn.predict(nieunTest0)
print("nieunTest0Pred: ", nieunTest0Pred) # Fails!

# Test new digeud
accumulative = np.array([0,1,3,5,8,8])
columnIdx = np.array([3,1,2,1,4,1,2,3])
values = np.ones(len(columnIdx))
digeudTest0 = csr_matrix((values,
                             columnIdx,
                             accumulative),
                         shape=letterImageSizePixels)
print("digeudTest0:\n", digeudTest0.toarray())
digeudTest0 = digeudTest0.reshape((1,-1))
digeudTest0Pred = knn.predict(digeud0)
print("digeudTest0Pred: ", digeudTest0Pred) # Fails!

# Because humans extract features (corners, number of sides of letters), but machines do not have that context, machines
# fails.
