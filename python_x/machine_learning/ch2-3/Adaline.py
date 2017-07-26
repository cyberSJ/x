#!/usr/bin/python
# Ch2-3. ADAptive LInear NEuron (Adaline) classifier machine learning algorithm.
# In machine learning in general, there are 3 key functions:
#   1. Classification function.
#       This determines the label given an input. It is used in both training
#       and real situation.
#   2. Update function.
#       This determines how the machine learning algorithm will learn. Each
#       machine learning rul differs in how the trained state is updated.
#   3. Cost function.
#       This defines the goal for the training.
# Using these 3 key functions, the machine learning algorithm is complete for
# implementation.

import numpy as np

class Adaline( object ):
    """ ADAptive LInear NEuron (Adaline) classifier machine learning 
        algorithm.
    """

    # learning_rate: How fast each update should accelerate going down the cost
    #     function.
    # iteration: number of iterations to repeat learning the entire training
    #     set.
    def __init__( self, learning_rate, iteration ):
        self.learning_rate = learning_rate
        self.iteration = iteration

    # X: NxM matrix representing the training input samples.
    #    N: # input samples.
    #    M: # features.
    # y: Nx1 vector represneting the classication of input samples.
    #    N: # classified output. Must be same as X's N.
    # underscoresed self variables means it is changed by public functions.
    def learn( self, X, y ):

        # Weigths have M+1 members because of the w0, which is defined withtout
        # a reason in the book for the Adaline algorithm. I will see if I
        # really need that w0.
        # sung What happens if the w0 is not present?
        self.weights_ = np.zeros( X.shape[1] + 1 )

        # Save costs for each iteration for graphing purpose. This is also a
        # good practice to see if the algorithm converged after the training.
        self.costs_ = []

        # sung what happens when there is only one iteration?
        for _ in range(self.iteration):

            # Adaline is a batch learning algorithm, which means it needs all
            # input before defining one set of trained system. Also this means
            # we don't need to compute one sample at a time, but instead
            # apply the classification function to all samples at once.

            # Classify all the input samples.
            # phi(z) = phi(wT*x) = wT*x for Adaline algorithm.
            classification = self.netInput( X )

            # Update the weights according to the errors.
            # w := w + delta(w) 
            #   := w + learning_rate * cost_function_derivative
            #
            # y - classification: N x 1 vector
            # X: N x M matrix
            # X.T: M x N matrix
            # so X.T.dot(y - classification): M x 1 vector = cost_function_derivative
            # cost_function_derivative: 1 x M vector. This is the key in
            # gradient decent.
            errors = y - classification
            cost_function_derivative = X.T.dot( errors )

            self.weights_[1:] += self.learning_rate * cost_function_derivative

            # w0 := w0 + delta(w0)
            #    := w0 + learning_rate * cost_function_derivative
            #    := w0 + learning_rate * dot(ones(Mx1),  errors)
            #    := w0 + learning_rate * sum(errors)
            self.weights_[0] += self.learning_rate * errors.sum()

            # Calculate cost for each learning iteration graphing purpose.
            cost = 0.5 * (errors**2).sum()
            self.costs_.append( cost )

        # Return object so that the user of this algorithm can netInput a new
        # input.
        return self

    # Compute the net sum of the weight
    # X: N x M matrix representing all N input samples with M features.
    # returns N x 1 vector representing net input for all N samples.
    def netInput( self, X ):

        # net_input will be ( N x M ) * ( M x 1 ) = N x 1. Where N can be 1 or
        # large.
        # After computing the doct product, the w0 will be added to
        # all samples because w0x0 = w0 * 1 = w0, where x0 is constant
        # threshold, which the book doesn't explain very well in the Adaline
        # algorithm section.
        net_input = np.dot( X, self.weights_[1:] ) + self.weights_[0]

        return net_input 

    # Quantizer is used only in the new input sample so that final
    # classification can occur. The training part of the Adaline algorithm does
    # not use the final classification, but uses raw result numerical value to
    # update the weights according to how far away the raw result was from the
    # prediction value.
    #
    # X: N x M matrix representing input samples.
    # returns the classification 1 or -1.
    def classify( self, X ):
        return np.where( self.netInput(X) >= 0.0, 1, -1)
