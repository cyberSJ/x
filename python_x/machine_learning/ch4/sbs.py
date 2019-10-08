#!/usr/bin/env python
# Sequential Backward Selection (SBS) algorithm.
# This algorithm performs feature selection, which means reducing the feature
# space by removing less-effective features.

from sklearn.base import clone
from itertools import combinations
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score


# This class does not calculate the score after the feature is reduced if the
# remaining feature equals the k_features. In other words, the k_feature is
# exclusive lower bound.
class SBS():
    def __init__( self,
                  estimator, # Machine learning algorithm to use.
                  k_features, # #features.
                  scoring = accuracy_score, # Built-in comparison.
                  test_size = 0.25,
                  random_state = 1 ):
        self.scoring = scoring
        self.estimator = clone(estimator) # Defensive-copy.
        self.k_features = k_features
        self.test_size = test_size
        self.random_state = random_state

    # fit() will use the classifier that was specifiec during the construction
    # of this class to learn the weights of the machine learning model.
    def fit( self,
             X,
             y ):

        # X_train: n x m matrix for sample data. n = number of samples. m =
        # number of features.
        X_train, X_test, y_train, y_test = \
            train_test_split( X,
                              y,
                              test_size = self.test_size,
                              random_state = self.random_state )

        # Get the number of features in the training set. This initializes the
        # currently selected number of features. If this value becomes less than
        # or equal to the user-defined k_features, the algorithm will exit.
        dim = X_train.shape[1]

        # indices = [0, #features]. Making the array to tuple means the
        # components of the data within the tuple is not modifiable.
        self.indices_ = tuple( range(dim) )

        # Current subset initialization. Create (and not reference) a new array.
        # So currently, self.substes_ is [0, #all_features]
        self.subsets_ = [self.indices_]

        # Calculate the score of the optimization function J using the current
        # subset.
        score = self._calc_score( X_train,
                                  y_train,
                                  X_test,
                                  y_test,
                                  self.indices_ )

        # Initialize the scores_ data structure.
        self.scores_ = [score]

        while dim > self.k_features:
        # {
            scores = []
            subsets = []

            # p will be a vector containing dim-1 features. Using all possible 
            # combinations of features with <dim - 1> number of features...
            for p in combinations( self.indices_, r = dim - 1 ):
            # {

                # ... calculate the score of the classifier with only those
                # features.
                score = self._calc_score( X_train,
                                          y_train,
                                          X_test,
                                          y_test,
                                          p )

                # Accumulate all scores for current number of features.
                scores.append( score )

                # Save current subset of features so that we can pick the best
                # among the saved subsets. subsetsx will be i x r matrix after
                # the for loop. i: Number of possible combinations. r: dim - 1.
                subsets.append( p )
            # }

            # Determine the subset that gave the best trained classifier.
            # For example, if the best score was given by the 3rd permutation,
            # the best_index will be 3 - 1 = 2 for the index.
            best_index = np.argmax( scores )

            # Discard the one feature that caused the worst classification.
            # self.indices_ will be a vector of length r = dim - 1.
            self.indices_ = subsets[ best_index ]

            # Each item that is appended does not need to match the dimension of
            # the previous items. Python list allows that. This is not a numpy
            # array. Each subsets_ element will contain the indicies for the
            # best chose features.
            self.subsets_.append( self.indices_ )

            dim -= 1

            # Store the score for using only the current set of features.
            self.scores_.append( scores[best_index] )

        # }

        # Reason for returning self is...?
        return self

    def _calc_score( self,
                     X_train,
                     y_train,
                     X_test,
                     y_test,
                     indices ):

        # Only train the classifier using the selected features.
        # Like Matlab, indices can be an array of index (not continuous).
        self.estimator.fit( X_train[:, indices], y_train )

        # Using the trained classifier, label all the test samples.
        # For the test data, we must use only the same type of features as used
        # in the training set. It doesn't really make sense to predict a test
        # data that has some other features because the trained classifier does
        # not have the weight defined for that un-relevant feature.
        y_pred = self.estimator.predict( X_test[:, indices] )

        # Compare the prediction with the real label to calculate the score of
        # this trained classifier with only the selected features. The scoring
        # algorithm was initialized during the constructor of tis class.
        score = self.scoring( y_test, y_pred )

        return score
