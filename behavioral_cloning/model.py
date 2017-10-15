import os
import csv
from random import shuffle
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Convolution2D, Activation, MaxPooling2D, Dropout
from sklearn.model_selection import train_test_split

import random
import numpy as np


def gaussian_blur(img, kernel_size=3):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    #return cv2.medianBlur(img, kernel_size)


center_count = 0;
samples = []
#identify first run
init = True

with open('data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        if(init != True):
            center_angle = float(line[3])
            if (center_angle == 0):
                center_count = center_count+1;
            if(center_angle != 0 or (center_angle == 0 and center_count%2==0)):
                samples.append(line)
        else:
            init = False

#samples = samples[1:1000]


train_samples, validation_samples = train_test_split(samples, test_size=0.3)

import cv2
import sklearn

#normalization (had to be outside of the NN model as the Lambda layer led to issues on my local machine )
normalize = lambda x: x / 255.0 - 0.5
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                name = 'data/IMG/'+batch_sample[0].split('/')[-1]
                ############## resubmission change ###############
                center_image_temp = cv2.imread(name)
                center_image = cv2.cvtColor(center_image_temp, cv2.COLOR_BGR2RGB)
                ###################################################
                center_angle = float(batch_sample[3])
                # apply gaussian blur
                blur= gaussian_blur(center_image)
                #normalize and add to list
                images.append(normalize(blur))
                angles.append(center_angle)
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

            
# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=32)
validation_generator = generator(validation_samples, batch_size=32)

# NN architecture
model = Sequential()
model.add(Cropping2D(cropping=((70, 25), (0, 0)), input_shape=(160, 320, 3)))
model.add(Convolution2D(24, 5, 5, subsample=(2,2), activation=("relu")))
model.add(Convolution2D(36, 5, 5, subsample=(2,2), activation=("relu")))
model.add(Convolution2D(48, 5, 5, subsample=(2,2), activation=("relu")))
model.add(Convolution2D(64, 3, 3, activation=("relu")))
model.add(Convolution2D(64, 3, 3, activation=("relu")))
model.add(Flatten())
model.add(Dense(100))
model.add(Dropout(0.5))
model.add(Dense(50))
model.add(Dropout(0.5))
model.add(Dense(10))
model.add(Dropout(0.5))
model.add(Activation('relu'))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')
print("First run. Learn general behavior.")
model.fit_generator(train_generator, samples_per_epoch=len(train_samples), validation_data=validation_generator,nb_val_samples=len(validation_samples), nb_epoch=10)


model.save('model_pre.h5')

#additional data set
center_count = 0;
samples = []
init = True

with open('select/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

#ignore heading 
samples = samples[1:]


train_samples, validation_samples = train_test_split(samples, test_size=0.3)

import cv2
import sklearn

normalize = lambda x: x / 255.0 - 0.5
def generator_additional(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                name = 'select/IMG/'+batch_sample[0].split('/')[-1]
                ############## resubmission change ###############
                center_image_temp = cv2.imread(name)
                center_image = cv2.cvtColor(center_image_temp, cv2.COLOR_BGR2RGB)
                ###################################################
                center_angle = float(batch_sample[3])
                blur= gaussian_blur(center_image)
                images.append(normalize(blur))
                angles.append(center_angle)
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

            

train_generator = generator_additional(train_samples, batch_size=32)
validation_generator = generator_additional(validation_samples, batch_size=32)

#retrain model with additional data
print("Second run. Finetuning of weights based on additional training data.")
model.fit_generator(train_generator, samples_per_epoch=len(train_samples), validation_data=validation_generator,nb_val_samples=len(validation_samples), nb_epoch=10)

#store final model
model.save('model.h5')