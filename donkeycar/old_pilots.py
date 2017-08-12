'''

pilots.py

Methods to create, use, save and load pilots. Pilots
contain the highlevel logic used to determine the angle
and throttle of a vehicle. Pilots can include one or more
models to help direct the vehicles motion.

'''
import os
import math
import random
from operator import itemgetter
from datetime import datetime

import numpy as np
import tensorflow as tf
import keras

import donkeycar.utils as utils

class BasePilot():
    '''
    Base class to define common functions.
    When creating a class, only override the funtions you'd like to replace.
    '''
    def __init__(self, name=None, last_modified=None):
        self.name = name
        self.last_modified = last_modified

    def decide(self, img_arr):
        angle = 0.0
        speed = 0.0
        return angle, speed, 'NaN'

    def load(self):
        pass


class KerasCategorical(BasePilot):
    def __init__(self, model_path, **kwargs):
        self.model_path = model_path
        self.model = None #load() loads the model
        super().__init__(**kwargs)

    def decide(self, img_arr):
        img_arr = img_arr.reshape((1,) + img_arr.shape)
        l = self.model.predict(img_arr)
        if len(l) == 3:
            angle_binned, throttle, speed = l
            speed = speed[0][0]
        else:
            angle_binned, throttle = l
            speed = 0.0
        # angle_binned, throttle = self.model.predict(img_arr)
        angle_certainty = max(angle_binned[0])
        angle_unbinned = utils.unbin_Y(angle_binned)
        return float(angle_unbinned[0]), float(throttle[0][0]), float(speed)

    def load(self):
        self.model = keras.models.load_model(self.model_path)
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        # print("Model= ")
        # self.model.summary()


class PilotHandler():
    """
    Convenience class to load default pilots
    """
    active_pilot = None

    def __init__(self, models_path = '~/mydonkey/models'):
        self.models_path = os.path.expanduser(models_path)
        self.active_pilot = None

    def pilots_from_models(self):
        """ Load pilots from keras models saved in the models directory. """
        from scandir import scandir
        models_list = [f for f in scandir(self.models_path)]
        pilot_list = []
        for d in models_list:
            last_modified = datetime.fromtimestamp(d.stat().st_mtime)
            pilot = KerasCategorical(d.path, name=d.name, last_modified=last_modified)
            pilot_list.append(pilot)

        print (pilot_list)
        return pilot_list

    def default_pilots(self):
        """ Load pilots from models and add CV pilots """
        pilot_list = self.pilots_from_models()
        #pilot_list.append(OpenCVLineDetector(name='OpenCV'))
        return pilot_list

    def run(self, img_arr, throttle, angle, speed, brake):
        if PilotHandler.active_pilot == None:
            return throttle, angle, speed
        else:
            if brake:
                return 0.0, 0.0, 0.0
            else:
                with PilotHandler.active_pilot.graph.as_default():
                    a, t, s =  PilotHandler.active_pilot.decide(img_arr)
                    print("pilot values= t= %s(%s)  a= %s(%s)  s= %s(%s)" % (type(t), str(t), type(a), str(a), type(s), str(s)))
                    return t, a, s

    def shutdown(self):
        # indicate that the thread should be stopped
        print('stopping PilotHandler')
