import mglearn 
# Generate a synthetic dataset called "forge".
#X, y = mglearn.datasets.make_forge() 
#
## Plot the dataset
#mglearn.discrete_scatter(X[:, 0], # First feature values on the x-axis.
#                         X[:, 1], # Second feature values on the y-axis.
#                         y) # Classification at each points in the xy plain.
#
#import matplotlib.pyplot as plt
#plt.legend(["Class 0", "Class 1"],
#           loc = 'lower right')
#plt.xlabel("Feature 0")
#plt.ylabel("Feature 1")
##plt.show()
#
#print ("X shape: ", X.shape)

#=======================================================================================================================

#X, y = mglearn.datasets.make_wave(n_samples=40)
#plt.plot(X, y, 'o')
#plt.ylim(-3, 3)
#plt.xlabel("Only one feature")
#plt.ylabel("Target")
#plt.show()

#=======================================================================================================================

#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#print("cancer.keys():\n", cancer.keys())
#print("data shape: ", cancer.data.shape)
#print("target names: ", cancer.target_names)
#print("target shape: ", cancer.target.shape)
#
#import numpy as np
#print("target bin count: ", np.bincount(cancer.target))
#print("Sample count per class: ",
#      {sample_name : count for sample_name, count in zip(cancer.target_names, np.bincount(cancer.target))})

#======================================================================================================================

#from sklearn.datasets import load_boston
#boston = load_boston()
#print("Data shape: ", boston.data.shape)
#X, y = mglearn.datasets.load_extended_boston()
#print("X.shape: ", X.shape)
#
#mglearn.plots.plot_knn_classification(n_neighbors=3)
#plt.show()

#======================================================================================================================
#X, y = mglearn.datasets.make_forge()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(X, y, random_state = 0)
#
#from sklearn.neighbors import KNeighborsClassifier
#knn = KNeighborsClassifier(n_neighbors = 3)
#knn.fit(XTrain, yTrain)
#print("Test set predictions: ", knn.predict(XTest))
#print("Test set score: {:.2f}".format(knn.score(XTest, yTest)))
#
#fig, axes = plt.subplots(1, 3)
#
#for neighborCount, axis in zip([1, 3, 9], axes):
#    knn = KNeighborsClassifier(n_neighbors = neighborCount).fit(X, y)
#    mglearn.plots.plot_2d_separator(knn, X, fill=True, eps=0.5, ax=axis, alpha=0.4)
#    mglearn.discrete_scatter(X[:, 0], X[:, 1], y, ax=axis)
#    axis.set_title("{} neighbor(s)".format(neighborCount))
#    axis.set_xlabel("feature 0")
#    axis.set_ylabel("feature 1")
#axes[0].legend(loc = "lower left")
#plt.show()

#======================================================================================================================

#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#print("cancer data type: ",  type(cancer.data))
#
#from sklearn.model_selection import train_test_split
#XTest, XTrain, yTest, yTrain = train_test_split(cancer.data,
#                                                cancer.target,
#                                                stratify = cancer.target,
#                                                random_state = 66)
#
#trainingAccuracy = []
#testAccuracy = []
#neighborSetting = range(1, 11)
#
#from sklearn.neighbors import KNeighborsClassifier
#for neighborCount in neighborSetting:
#    knn = KNeighborsClassifier(n_neighbors = neighborCount)
#    knn.fit(XTrain, yTrain)
#    trainingAccuracy.append(knn.score(XTrain, yTrain))
#    testAccuracy.append(knn.score(XTest, yTest))
#
#import matplotlib.pyplot as plt
#plt.plot(neighborSetting, trainingAccuracy, label = "Training accuracy")
#plt.plot(neighborSetting, testAccuracy, label = "Test accuracy")
#plt.ylabel("Accuracy")
#plt.xlabel("Neighbor count")
#plt.legend()
#plt.show()
#======================================================================================================================

#import mglearn
#import matplotlib.pyplot as plt
#mglearn.plots.plot_knn_regression(n_neighbors = 3)
#plt.show()
#======================================================================================================================

from sklearn.neighbors import KNeighborsRegressor
X, y = mglearn.datasets.make_wave(n_samples=40)

from sklearn.model_selection import train_test_split
XTrain, XTest, yTrain, yTest = train_test_split(X, y, random_state = 0)

reg = KNeighborsRegressor(n_neighbors = 3)

reg.fit(XTrain, yTrain)
print("Test set prediction:\n", reg.predict(XTest))
print("Test set R^2: {:.2f}".format(reg.score(XTest, yTest)))

import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 3, figsize=(15,4)) # sung play with the figsize.

import numpy as np
line = np.linspace(-3, 3, 1000).reshape(-1, 1) # Reshape to a column vector.

for neighborCount, ax in zip([1, 3, 9], axes):
    reg = KNeighborsRegressor(n_neighbors = neighborCount)
    reg.fit(XTrain, yTrain)
    ax.plot(line, reg.predict(line))
    ax.plot(XTrain, yTrain, '^', c=mglearn.cm2(0), markersize = 8)
    ax.plot(XTest, yTest, 'v', c=mglearn.cm2(1), markersize = 8)

    ax.set_title("{} neighbor(s)\ntrain score: {:.2f} test score: {:.2f}".format(
                    neighborCount,
                    reg.score(XTrain, yTrain),
                    reg.score(XTest, yTest)))

    ax.set_xlabel("Feature space")
    ax.set_ylabel("Prediction")

axes[0].legend(["Model predictions",
                "Train input & target",
                "Test input & target"],
               loc = "best")
plt.show()
