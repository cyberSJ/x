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

import mglearn
X, y = mglearn.datasets.load_extended_boston()

from sklearn.model_selection import train_test_split
XTrain, XTest, yTrain, yTest = train_test_split(X, y, random_state = 0)

#from sklearn.linear_model import LinearRegression
#lr = LinearRegression().fit(XTrain, yTrain)
#
#print("Train set score: {:.2f}".format(lr.score(XTrain, yTrain)))
#print("Test set scroe : {:.2f}".format(lr.score(XTest, yTest)))
# =================================================================================================

from sklearn.linear_model import Ridge
ridge = Ridge(alpha=0.1).fit(XTrain, yTrain)
print("Train set score: {:.2f}".format(ridge.score(XTrain, yTrain)))
print("Test set score : {:.2f}".format(ridge.score(XTest, yTest)))
