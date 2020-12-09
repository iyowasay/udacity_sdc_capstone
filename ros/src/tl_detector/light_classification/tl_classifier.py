from styx_msgs.msg import TrafficLight
import os
import cv2
import numpy as np
import rospy
import tensorflow as tf

from keras.models import Sequential, load_model
from keras.layers import Dense, Activation, Dropout, Flatten, Reshape
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        model = Sequential()

        model.add(Conv2D(filters, kernel_size, strides=(1, 1), padding='valid', data_format=None, dilation_rate=(1, 1), activation=None, use_bias=True, kernel_initializer='glorot_uniform', bias_initializer='zeros', kernel_regularizer=None, bias_regularizer=None, activity_regularizer=None, kernel_constraint=None, bias_constraint=None))
        model.add(MaxPooling2D(pool_size=(2, 2), strides=None, padding='valid', data_format=None))
        model.add(Dropout(0.75))
        model.add(Flatten())

        model.add(Dense(units=64, input_dim=100))
        model.add(Activation('relu'))
        model.add(Dense(units=64, input_dim=100))
        model.add(Activation('relu'))
        model.add(Dense(units=64, input_dim=100))
        model.add(Activation('relu'))


        model.compile(loss='categorical_crossentropy', optimizer='sgd', metrics=['accuracy'])
        model.fit(X_train, Y_train, epochs=10, batch_size =32) 

        loss_and_metrics = model.evaluate(x_test, y_test, batch_size=128)
        classes = model.predict(x_test, batch_size=128)

        # uint8 UNKNOWN=4
        # uint8 GREEN=2
        # uint8 YELLOW=1
        # uint8 RED=0

        
        return TrafficLight.UNKNOWN
