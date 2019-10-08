#!/usr/bin/python
# Online (updating the learning sytem with real-time one sample) version of the
# ADAptive LInear NUeron machine learning algorithm.

import numpy as np
from numpy.random import seed

class AdalineOnline( object ):
    
    def __init__( self, 
                  learning_rate, 
                  iterations,
                  random_state_seed = 1 ):
        self.learning_rate = learning_rate
        self.iterations = iterations
        self.weights_initialized = False
        seed( random_state_seed )

    # Learns the Adaline weight through the data X and label y.
    # X: N x M matrix representing N input samples with M features.
    # y: N x 1 vector representing the label of N input samples.
    def stochasticGradientDescent( self, X, y ):

        # weights_: M x 1 vector representing weigths to multiple for each
        # feature value.
        self.weights_ = np.zeros(X.shape[1] + 1)
        self.weights_initialized = True 
        self.avg_costs_ = []

        for _ in range( self.iterations ):

            # Shuffle the mini-batch input samples so that each iteration can
            # search for potential multiple local minima in the cost function.
            shuffled_idx = np.random.permutation(len(y))
            X_shuffled = X[ shuffled_idx ]
            y_shuffled = y[ shuffled_idx ]

            # Generate the output using the classification guide function.
            # We are using a mini-batch algorithm for learning.
            # sung why are we updating the weight after each sample if we are
            # using mini-batch learning? Are we using pure online learning?
            costs = []
            for Xi, label in zip( X_shuffled, y_shuffled ):
                costs.append( self._updateWeights(Xi, label) )

            # Calculate the average cost up to this iteration. The average
            # cost will be the measure of how we progress in a online
            # learning algorithm.
            self.avg_costs_.append( sum(costs) / len(costs) )
            print "sung hey"

        # Return self so that external code can use the public functions to
        # classify a real data.
        return self

    # Learns withtout re-initializing the weights.
    # X: N x M matrix for N input samples, each with M features.
    # y: N x 1 matrix for N labels on the input samples.
    def onlineLearn( self, X, y ):
        # Initialize the weights if not initialized
        if not self.weights_initialized:
            self.weights_ = np.zeros( X.shape[1] + 1 )

        num_labels = y.ravel().shape[0]

        if num_labels <= 0:
            print """Number of labels must be positive. Bailing out
                  partialLearn()..."""
            return None

        if num_labels >= 1:
            # Learn with each sample
            for Xi, label in zip( X, y ):
                self._updateWeights( Xi, label )

        return self

    # X: N x M matrix representing N input samples with M features.
    # returns the classification label (1 or -1)
    def classify( self, X ):
        net_input = self._netInput( X )
        return np.where( net_input >= 0.0, 1, -1 )

    # x: 1 x M vector for one input sample with M features.
    # y: Integer for the true label for the input.
    # returns the cost of the prediction agianst the truth.
    def _updateWeights( self, x, y ):
        # Make prediction for the label on the input
        net_input = self._netInput( x );
        error = y - net_input

        # Update the weight
        # w := w + delta(w)
        #    = w + learning_rate * cost_function_derivative
        #    = w + learning_rate * error * input_sample
        self.weights_[1:] += self.learning_rate * x.dot( error )
        self.weights_[0] += self.learning_rate * error * 1

        cost = 0.5 * error**2
        return cost

    # X: N x M vector for one input sample with M features.
    # returns the raw classification value.
    def _netInput( self, X ):
        return np.dot( X, self.weights_[1:] ) + self.weights_[0]
