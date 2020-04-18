#import mglearn
#import matplotlib.pyplot as plt
#
##mglearn.plots.plot_linear_regression_wave()
##plt.show()
#
#from sklearn.linear_model import LinearRegression
#X, y = mglearn.datasets.make_wave(n_samples = 60)
#
##mglearn.discrete_scatter(X, y)
##plt.show()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest =  train_test_split(X, y, random_state = 42)
#lr = LinearRegression().fit(XTrain, yTrain)
#print("lr.coef_: ", lr.coef_)
#print("lr.intercept_: ", lr.intercept_)
#print("Training set score: {:.2f}".format(lr.score(XTrain, yTrain)))
#print("Test set score: {:.2f}".format(lr.score(XTest, yTest)))
# =================================================================================================

#import mglearn
#X, y = mglearn.datasets.load_extended_boston()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(X, y, random_state = 0)
#print("XTrain shape: ", XTrain.shape)
#print("XTrain: ", XTrain)
#print("yTrain shape: ", yTrain.shape)
#print("yTrain: ", yTrain)

#from sklearn.linear_model import LinearRegression
#lr = LinearRegression().fit(XTrain, yTrain)
#
#print("Train set score: {:.2f}".format(lr.score(XTrain, yTrain)))
#print("Test set scroe : {:.2f}".format(lr.score(XTest, yTest)))
# =================================================================================================

#from sklearn.linear_model import Ridge
#ridge01 = Ridge(alpha=0.1).fit(XTrain, yTrain)
#print("Train set score: {:.2f}".format(ridge01.score(XTrain, yTrain)))
#print("Test set score : {:.2f}".format(ridge01.score(XTest, yTest)))
#
#ridge1 = Ridge(alpha=1).fit(XTrain, yTrain)
#ridge10 = Ridge(alpha=10).fit(XTrain, yTrain)
#
#from sklearn.linear_model import LinearRegression
#lr = LinearRegression().fit(XTrain, yTrain)
#
#import matplotlib.pyplot as plt
#plt.plot(ridge1.coef_, "s", label="Ridge alpha = 1")
#plt.plot(ridge01.coef_, '^', label='Ridge alpha = 0.1')
#plt.plot(ridge10.coef_, 'v', label = 'Ridge alpha = 10')
#plt.plot(lr.coef_, 'o', label = 'Linear regression')
#plt.xlabel('Coefficient index. There are this many coefficients in the regression equation.')
#plt.ylabel('Coefficient magnitude')
#plt.ylim(-25, 25)
#plt.hlines(0, 0, len(lr.coef_))
#plt.legend(loc = 'best')
#plt.show()
# =================================================================================================

#from sklearn.linear_model import Lasso
#lasso = Lasso().fit(XTrain, yTrain)
#print("Training score: {:.2f}".format(lasso.score(XTrain, yTrain)))
#print("Test set score: {:.2f}".format(lasso.score(XTest, yTest)))
#
#import numpy as np
#print("Number of features used: ", np.sum(lasso.coef_ != 0))
#
#lasso001 = Lasso(alpha = 0.01, max_iter = 1e5).fit(XTrain, yTrain)
#print("Training score lasso001: {:.2f}".format(lasso001.score(XTrain, yTrain)))
#print("Test set score lasso001: {:.2f}".format(lasso001.score(XTest, yTest)))
#print("Number of feature used lasso001: ", np.sum(lasso001.coef_ != 0))

# =================================================================================================

#import mglearn
#X, y = mglearn.datasets.make_forge()
#print("X shape: ", X.shape)
#print("X:\n", X)
#print("y shape: ", y.shape)
#print("y:\n", y)
#
#import matplotlib.pyplot as plt
#fig, axes = plt.subplots(1, 2, figsize=(10, 3))
#
#from sklearn.svm import LinearSVC
#from sklearn.linear_model import LogisticRegression
#
#for model, ax in zip([LinearSVC(), LogisticRegression()], axes):
#    classifier = model.fit(X, y)
#    mglearn.plots.plot_2d_separator(classifier,
#                                    X,
#                                    fill=False,
#                                    eps=0.5, # sets the zoom of the graph.
#                                    ax=ax, # This is a subplot.
#                                    alpha=0.3) # gradient
#
#    mglearn.discrete_scatter(X[:, 0], X[:, 1], y, ax=ax)
#    ax.set_title(classifier.__class__.__name__)
#    ax.set_xlabel("Feature 0")
#    ax.set_ylabel("Feature 1")
#
#axes[0].legend()
#plt.show()
#
#from sklearn.model_selection import train_test_split
# =================================================================================================
#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(cancer.data, 
#                                                cancer.target, 
#                                                stratify=cancer.target, 
#                                                random_state=42)
#
#print("X shape: ", XTrain.shape)
#print("y shape: ", yTrain.shape)
#from sklearn.linear_model import LogisticRegression
#classifier = LogisticRegression(C=100).fit(XTrain, yTrain)
#print("Training score: {:.3f}".format(classifier.score(XTrain, yTrain)))
#print("Test set score: {:.3f}".format(classifier.score(XTest, yTest)))
#
#import matplotlib.pyplot as plt
#plt.plot(classifier.coef_.T, 'o', label="C=1")
#
## Label each point in the x-axis, which are the individual features.
#plt.xticks(range(cancer.data.shape[1]), # [1 - 30], the number of features
#           cancer.feature_names,
#           rotation=90)
#plt.hlines(0 ,0, cancer.data.shape[1]) # draw a axis that is 30 units long.
#plt.xlabel("Features")
#plt.ylabel("Coefficient magnitude")
#plt.legend()
#plt.clf()
#
#for cValue , marker in zip([0.001, 1, 100], ['o', '^', 'v']):
#    lrl1 = LogisticRegression(C=cValue, penalty='none').fit(XTrain, yTrain)
#    plt.plot(lrl1.coef_.T, marker, label="C={:.3f}".format(cValue))
#
#plt.show()
# =================================================================================================
#
#from sklearn.datasets import make_blobs
#X, y = make_blobs(random_state=42)
#print("X shape: ", X.shape)
#print("y shape: ", y.shape)
#
#import mglearn
#mglearn.discrete_scatter(X[:,0], X[:,1], y)
#
#import matplotlib.pyplot as plt
#plt.xlabel("Feature 0")
#plt.ylabel("Feature 1")
#plt.legend(["Class 0", "Class 1", "Class 2"])
#
#from sklearn.svm import LinearSVC
#svm = LinearSVC().fit(X, y)
#print("Coefficient shape: ", svm.coef_.shape)
#print("Intercept shape: ", svm.intercept_.shape)
#
#import numpy as np
#line = np.linspace(-15, 15) # x-axis?
#for coef, intercept, color in zip(svm.coef_, svm.intercept_, mglearn.cm3.colors):
#        plt.plot(line, -(line * coef[0] + intercept) / coef[1], c=color)
#
#plt.ylim(-10, 15)
#plt.xlim(-10, 8)
#plt.legend(['Class 0', 'Class 1', 'Class 2', 'Line class 0', 'Line class 1', 'Line class 2'], loc='best')
#
#mglearn.plots.plot_2d_classification(svm, X, fill=True, alpha=0.4)
#
#plt.show()
# =================================================================================================
#
#from sklearn.tree import DecisionTreeClassifier
#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(cancer.data,
#                                                cancer.target,
#                                                stratify=cancer.target,
#                                                random_state=42)
#print("XTrain shape: ", XTrain.shape)
#print("yTrain shape: ", yTrain.shape)
#dt = DecisionTreeClassifier(max_depth = 4, random_state = 0)
#dt.fit(XTrain, yTrain)
#print("Accuracy on traiing set: {:.3f}".format(dt.score(XTrain, yTrain)))
#print("Accuracy on test set   : {:.3f}".format(dt.score(XTest, yTest)))
#
## For some reason, sklearn.tree.plot_tree() image is not zommable in Linux..T_T
##import sklearn
##print("sklearn version: ", sklearn.__version__)
##
##sklearn.tree.plot_tree(dt, filled = True)
##
##import matplotlib.pyplot as plt
##plt.show()
#
## Plot using graphviz instead of sklearn.tree.ploot_tree().
#from sklearn.tree import export_graphviz
#export_graphviz(dt,
#                out_file="tree.dot",
#                class_names=['bad', 'good'],
#                feature_names=cancer.feature_names,
#                impurity=False,
#                filled=True)
#
#import graphviz
#with open("tree.dot") as f:
#    dotGraphFile = f.read()
#    graphviz.Source(dotGraphFile).render(filename="tree.dot",  # The original tree.dot above will be overwritten.
#                                         view=True)
#
#import matplotlib.pyplot as plt
#import numpy as np
#def plotFeatureImportancesForCancer(cancerBunch, model):
#    featureCount = cancerBunch.data.shape[1]
#    plt.barh(np.arange(featureCount), model.feature_importances_, align='center')
#    plt.yticks(np.arange(featureCount), cancerBunch.feature_names)
#    plt.xlabel("Feature importance")
#    plt.ylabel("Features")
#    plt.ylim(-1, featureCount)
#
#plotFeatureImportancesForCancer(cancer, dt)
#plt.show()
# =================================================================================================
#import os
#import pandas as pd
#import mglearn
#ramPrices = pd.read_csv(os.path.join(mglearn.datasets.DATA_PATH,
#                                     "ram_price.csv"))
##print("ramPrices: ", ramPrices)
##import matplotlib.pyplot as plt
##plt.semilogy(ramPrices.date, ramPrices.price)
##plt.xlabel("Year")
##plt.ylabel("$ per MB")
##plt.show()
#
## Must use () instead of [] to capture the indicies. [] makes the indices an array, where () makes the indices a list.
#trainIndicies = (ramPrices.date < 2000)
#print("trainIndicies: ", trainIndicies)
##dataTrain = [ramPrices[i] for i in trainIndicies]
#dataTrain = ramPrices[trainIndicies]
#print("dataTrain shape: ", dataTrain.shape)
#dataTest = ramPrices[trainIndicies == False]
#print("dataTest shape: ", dataTest.shape)
#
## predict prices based on date.
## Need to take the logarithm of the price because we wnat to model a linear relationship.
## The input must match the dimension of this logairthm'ed price.
## Why the input must be a 2D matrix? Because XTrain must be [samples x features] 2D matrix. And in this case the feature
## count is 1.
#import numpy as np
#yTrain = np.log(dataTrain.price)
##print("yTrain: \n", yTrain)
#
#XTrain = dataTrain.date[:, np.newaxis]
##print("XTrain: \n", XTrain)
#
#from sklearn.tree import DecisionTreeRegressor
#tree = DecisionTreeRegressor(max_depth=3).fit(XTrain, yTrain)
#
#from sklearn.linear_model import LinearRegression
#linear = LinearRegression().fit(XTrain, yTrain)
#
## Predict on all data (train and test) for drawing purpose.
#XAll = ramPrices.date[:, np.newaxis]
##print("XAll:\n", XAll)
#treePrediction = tree.predict(XAll)
#linearPrediction = linear.predict(XAll)
#
## Since the input to the models was in logaritm, the output needs to be converted to linear.
#treePrediction = np.exp(treePrediction)
#linearPrediction = np.exp(linearPrediction)
#
## Plot the result in the log scale.
#import matplotlib.pyplot as plt
#plt.semilogy(dataTrain.date, dataTrain.price, label="Training data")
#plt.semilogy(dataTest.date, dataTest.price, label="Test data")
#plt.semilogy(ramPrices.date, treePrediction, label="Tree prediction")
#plt.semilogy(ramPrices.date, linearPrediction, label="Linear prediction")
#plt.legend()
#plt.show()
#plt
# =================================================================================================
#from sklearn.datasets import make_moons
#X, y = make_moons(n_samples=100, noise=0.25, random_state=3)
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(X, y, stratify=y, random_state=42)
#
#from sklearn.ensemble import RandomForestClassifier
#randomForest = RandomForestClassifier(n_estimators=5, random_state=2)
#randomForest.fit(XTrain, yTrain)
#
#import matplotlib.pyplot as plt
#figure, plots = plt.subplots(2, 3, figsize=(20, 10))
#for index, (plot, tree) in enumerate(zip(plots.ravel(), randomForest.estimators_)):
#    plot.set_title("Tree index: {}".format(index))
#
#    import mglearn
#    mglearn.plots.plot_tree_partition(XTrain, yTrain, tree, ax=plot)
#
## The last 6th plot is the overall random forest that combines all trees.
#lastPlot = plots[-1, -1]
#mglearn.plots.plot_2d_separator(randomForest, XTrain, fill=True, ax=lastPlot, alpha=0.4)
#lastPlot.set_title("Random Forest")
#
## Plogt the train data onto the last plot.
#mglearn.discrete_scatter(XTrain[:, 0], XTrain[:, 1], yTrain)
#plt.show()
# =================================================================================================
#
#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(cancer.data, cancer.target, random_state=0)
#
#from sklearn.ensemble import RandomForestClassifier
#forest = RandomForestClassifier(n_estimators=100, random_state=0)
#forest.fit(XTrain, yTrain)
#
## Find the accuracy of the model.
#print("Train set accuracy: {:.3f}".format(forest.score(XTrain, yTrain)))
#print("Test set accuracy : {:.3f}".format(forest.score(XTest, yTest)))
#
## What to do with the predictions? Visualize the importances of features.
#import matplotlib.pyplot as plt
#import numpy as np
#def plotFeatureImportancesForCancer(cancerBunch, model):
#    featureCount = cancerBunch.data.shape[1]
#    plt.barh(np.arange(featureCount), width=model.feature_importances_)
#    plt.yticks(np.arange(featureCount), cancerBunch.feature_names)
#    plt.ylabel("Features")
#    plt.xlabel("Feature importance")
#    plt.ylim(-1, featureCount)
#    
#plotFeatureImportancesForCancer(cancer, forest)
#plt.show()
# =================================================================================================
#from sklearn.datasets import load_breast_cancer
#cancer = load_breast_cancer()
#
#from sklearn.model_selection import train_test_split
#XTrain, XTest, yTrain, yTest = train_test_split(cancer.data, cancer.target, random_state=0)
#
#from sklearn.ensemble import GradientBoostingClassifier
#gb = GradientBoostingClassifier(random_state=0, max_depth=1)
#gb.fit(XTrain, yTrain)
#
#print("Train accuracy  : {:.3f}".format(gb.score(XTrain, yTrain)))
#print("Testing accuracy: {:.3f}".format(gb.score(XTest, yTest)))
#
#import matplotlib.pyplot as plt
#import numpy as np
#def plotFeatureImportances(cancerBunch, model):
#    featureCount = cancerBunch.data.shape[1]
#    plt.barh(np.arange(featureCount), width=model.feature_importances_)
#    plt.yticks(np.arange(featureCount), cancerBunch.feature_names)
#
#plotFeatureImportances(cancer, gb)
#plt.show()
# =================================================================================================o

#from sklearn.datasets import make_blobs
#X, y = make_blobs(centers=4, random_state=8)
#y = y % 2
#print("X shape: ", X.shape)
#print("y:\n", y)
#
#import mglearn
##mglearn.discrete_scatter(X[:, 0], X[:, 1], y)
#
#import matplotlib.pyplot as plt
##plt.xlabel("Feature 0")
##plt.ylabel("Feature 1")
#
#from sklearn.svm import LinearSVC
#svm = LinearSVC().fit(X, y)
#
##mglearn.plots.plot_2d_separator(svm, X)
#
#import numpy as np
## tupleOfInputs: (100x2 matrix), 100x1 array). So there are 2 elements in the tuple.
#tupleOfInputs = [X, X[:, 1:] ** 2]
#
## hstack() can horizontally stack each element in the tuple. So the result becomes 100x3 matrix.
#XnewFeature = np.hstack(tupleOfInputs)
#
#from mpl_toolkits.mplot3d import Axes3D, axes3d
##figure = plt.figure()
#
##ax = Axes3D(fig=figure, elev=-152, azim=-26)
##
##class0Index = (y == 0)
##ax.scatter(XnewFeature[class0Index, 0],
##           XnewFeature[class0Index, 1],
##           XnewFeature[class0Index, 2],
##           c='blue',
##           cmap=mglearn.cm2,
##           s=60,
##           edgecolor='k')
##ax.scatter(XnewFeature[~class0Index, 0],
##           XnewFeature[~class0Index, 1],
##           XnewFeature[~class0Index, 2],
##           c='red',
##           marker='^',
##           cmap=mglearn.cm2,
##           s=60,
##           edgecolor='k')
##ax.set_xlabel("Feature 0")
##ax.set_ylabel("Feature 1")
##ax.set_zlabel("Feature 1 ** 2")
#
#svm = LinearSVC().fit(XnewFeature, y)
#
## Create an f0-axis value for the decision plane.
#f0AxisValues = np.linspace(XnewFeature[:, 0].min() - 2, XnewFeature[:,0].max() + 2, 50)
#print("f0AxisValues: \n", f0AxisValues)
#
## Create an f1-axis value for the decision plane.
#f1AxisValues = np.linspace(XnewFeature[:, 1].min() - 2, XnewFeature[:,1].max() + 2, 50)
#
## Apply the equation of plane to produce the f2-axis values.
## But first, we need the points in the f0f1-plane
#XX, YY = np.meshgrid(f0AxisValues, f1AxisValues)
#print("XX;\n", XX)
#print("YY;\n", YY)
#
## Now apply the equation of the plane. 
## coef[0] belongs to feature0.
## coef[1] belongs to feature1.
## The ravel() is needed because the original coef is 1x3 array, whereas raveled version is 3-element list, and we need a
## list because we want to index the coef array with just a single index. For the original 1x3 array, we have to use like
## coef[0,0], coef[0,1], coef[0,2]. In the 3-lement array, we can just access like coef[0], coef[1], and coef[2].
#coef = svm.coef_.ravel()
#
## intercept is needed for the equation of the plane.
#intercept = svm.intercept_
#ZZ = (coef[0] * XX + coef[1] * YY + intercept) / -coef[2]
##ax.plot_surface(XX, YY, ZZ, rstride=8, cstride=8, alpha=0.3)
#
#ZZ = YY ** 2
#locationRelativeToDecisionBoundary = svm.decision_function(np.c_[XX.ravel(), YY.ravel(), ZZ.ravel()])
#
#import matplotlib.pyplot as plt
#import mglearn
#plt.contourf(XX, YY, locationRelativeToDecisionBoundary.reshape(XX.shape),
#             levels=[locationRelativeToDecisionBoundary.min(), 0, locationRelativeToDecisionBoundary.max()],
#             cmap=mglearn.cm2,
#             alpha=0.5)
#
#mglearn.discrete_scatter(X[:,0], X[:,1], y)
#plt.xlabel("Feature 0")
#plt.ylabel("Feature 1")
#plt.show()
# =================================================================================================o

import mglearn
X, y = mglearn.tools.make_handcrafted_dataset()


def plotSvm(plot, samples, labels, importance, gamma):
    from sklearn.svm import SVC
    svm = SVC(kernel="rbf", C=importance, gamma=gamma).fit(X, y)

    mglearn.plots.plot_2d_separator(svm, samples, eps=0.5, ax=plot)

    mglearn.discrete_scatter(samples[:,0], samples[:,1], labels, ax=plot)

    # Select the feature-coordinates of the support vectors.
    supportVector = svm.support_vectors_

    # Re-Draw the support vector. This involves re-knowing their class labels.
    supportVectorClassLabels = svm.dual_coef_.ravel() > 0

    mglearn.discrete_scatter(supportVector[:,0], 
                             supportVector[:,1],
                             supportVectorClassLabels,
                             s=15,
                             markeredgewidth=3,
                             ax=plot)

    plot.set_title("C: {:.2f}  gamma: {:.2f}".format(importance, gamma))

import matplotlib.pyplot as plt
fig, axes = plt.subplots(3, 3)

for axRow, importance in zip(axes, [0.1, 1, 1e3]):
    for ax, gamma in zip(axRow, [0.1, 1, 10]):
        plotSvm(ax, X, y, importance, gamma)

plt.legend()
plt.show()
