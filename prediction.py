#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Library for short term predictions of future locations. Given the set of physical movement trajectories throughout 
the experiment, the idea is to evaluate the accuracy of predicting near-future physical locations. 
"""

__author__ = "Filip Lemic, Jakob Struye, Jeroen Famaey"
__copyright__ = "Copyright 2021, Internet Technology and Data Science Lab (IDLab), University of Antwerp - imec"
__version__ = "1.0.0"
__maintainer__ = "Filip Lemic"
__email__ = "filip.lemic@uantwerpen.be"
__status__ = "Development"

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import numpy as np
import pandas
import tensorflow as tf

np.random.seed(7)


# Given a Pandas DataFrame, this method will generate a training array X consisting of n_past 
# observations, as well as a testing array y of consisting of n_future predicted values  
def split_series(series, n_past, n_future):
	# n_past - number of past observations
	# n_future - number of future observations 

	X, y = list(), list()
	for window_start in range(len(series)):
		past_end = window_start + n_past
		future_end = past_end + n_future
	
		if future_end > len(series):
  			break

		# slicing the past and future parts of the window
		past, future = series[window_start:past_end, :], series[past_end:future_end, :]
		X.append(past)
		y.append(future)
	return np.array(X), np.array(y)


def make_and_evaluate_predictions(dataset, n_past, n_future, n_features, split_rate = 0.8):
	# n_past - number of past observations
	# n_future - number of future observations 
	# n_features - number of features to be predicted (usually 2, i.e., x and y coordinates)s

	# Dataset should be a Pandas DataFraame object
	dataset = pandas.DataFrame(dataset)
	dataset.index.name = 'id'

	# Divide into training and test sets
	train_df, test_df = dataset[0:int(len(dataset) * split_rate)], dataset[int(len(dataset) * split_rate):] 

	# Divide both training and test sets into chunks of observations. 
	X_train, y_train = split_series(train_df.values, n_past, n_future)
	X_test, y_test = split_series(test_df.values, n_past, n_future)


	encoder_inputs = tf.keras.layers.Input(shape=(n_past, n_features))
	encoder = tf.keras.layers.LSTM(60, activation = 'tanh', return_state=True)
	encoder_outputs = encoder(encoder_inputs)

	encoder_states = encoder_outputs[1:]

	decoder_inputs = tf.keras.layers.RepeatVector(n_future)(encoder_outputs[0])

	decoder = tf.keras.layers.LSTM(60, activation = 'tanh', return_sequences = True)(decoder_inputs, initial_state = encoder_states)
	decoder_outputs = tf.keras.layers.TimeDistributed(tf.keras.layers.Dense(n_features))(decoder)

	model = tf.keras.models.Model(encoder_inputs, decoder_outputs)

	reduce_lr = tf.keras.callbacks.LearningRateScheduler(lambda x: 1e-4 * 0.90 ** x)
	model.compile(loss = tf.keras.losses.MeanSquaredError(), optimizer = 'adam', metrics = ['mean_squared_error'])
	history = model.fit(X_train, y_train, epochs = 50, validation_data = (X_test, y_test), batch_size = 32, verbose = 0, callbacks = [reduce_lr])

	y_pred = model.predict(X_test)

	y_test = y_test.reshape((len(y_test), 2))
	y_pred = y_pred.reshape((len(y_pred), 2))

	mse = (np.square(y_test - y_pred)).mean(axis = 1)

	return mse.tolist()


