#!/usr/bin/python
# Studying from Python Machine Learning by Sebastian Raschka.
import numpy as np

# Why inherit object class? For following the "new-style" Python class.
# Nothing much difference for my purpose here.
class Perceptron(object):
    "Perceptron classifier. My first machine learning algorithm in Python."

    # args:
    #     eta: Float. Learning rate between 0.0 and 1.0.
    #
    #     n_iter: Integer. How many to repeat training. I guess you need to
    #     repeat because you fail the first ones pretty easily and need to
    #     re-train with the upgraded model after an iteration.
    def __init__(self, eta, n_iter):
        self.eta = eta
        self.n_iter = n_iter

    # Fits the 
    # args:
    #     X: N x M matrix of floats. N = number of samples. 
    #        M = number of features.
    #     y: N x 1 vector containing the actual classification. 
    #        N = number of samples.
    def fit(self, X, y):
        # initialize the weights to zero including the threshold weight.
        self.weights_ = np.zeros(1 + X.shape[1])
        # sung see what happens when we don't include the threshold weight.
        # sung what happens when the w0 weight is not 0?

        self.errors_ = []

        for _ in range(self.n_iter):

            # Count how many mis-predictions occurred for each iteration over 
            # all samples.
            errors = 0

            for xi, actual in zip(X, y):
                # xi is now a 1 X M vector where M is the number of features.
                # For our purose, the features are each of the Myelin sheath.
                # actual is a single value for the actual output.

                # Make prediction on classification
                prediction = self.predict( xi );
                
                # Apply the Perceptron rule:
                # sung How is this rule derived from? To me who does not know any
                # math behind it, it looks like some equation that kinda forces
                # the system to regress toward some target without very
                # thorough mathematical proof that the equation will work.
                update_value = self.eta * (actual - prediction)
                errors += int(update_value != 0.0)

                # After examining single input sample, update THE weights which
                # represents how well we learned.
                self.weights_[1:] += update_value * xi

                # Following is really update_value * 1 since we assume the
                # first element in xi is always 1.
                self.weights_[0] += update_value

                # Updating the weights is our act of "machine learning"
                # After updating the weights with all examination of training 
                # inputs, we are so called "well-trained".

            self.errors_.append( errors )
        return self
        # Why return self? To access the errors_ object later outside this
        # object.

    # xi: 1 x M vector representing a single sample. M = #features.
    def predict(self, xi):
        # Make prediction by summing all features with weight using numpy.
        z = np.dot(self.weights_[1:], xi) + self.weights_[0];
        prediction = np.where(z >= 0.0, 1, -1)
        return prediction
