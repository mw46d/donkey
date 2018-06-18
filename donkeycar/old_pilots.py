'''

pilots.py

Methods to create, use, save and load pilots. Pilots
contain the highlevel logic used to determine the angle
and throttle of a vehicle. Pilots can include one or more
models to help direct the vehicles motion.

'''
from datetime import datetime
import json
import kestrel
import math
import random
from operator import itemgetter
import os
import threading
import time

import numpy as np
import tensorflow as tf
import keras

import donkeycar.subscribers as subscribers
import donkeycar.utils as utils

from PIL import Image

NaN = float('nan')
RC = 1.0 / (2.0 * math.pi * 20.0)

class PID_Info:
    P = 0.0
    I = 0.0
    D = 0.0
    desired = 0.0

class PID:
    _Kp = 0.0
    _Ki = 0.0
    _Kd = 0.0
    _integrator = 0.0
    _last_derivative = 0.0
    _last_error = 0.0
    _last_t = 0
    _imax = 5000.0
    _pid_info = PID_Info()

    def __init__(self, p, i, d):
        self._Kp = p
        self._Ki = i
        self._Kd = d

    def reset_i(self):
        self._integrator = 0
        # we use NAN (Not A Number) to indicate that the last
        # derivative value is not valid
        self._last_derivative = NaN
        self._pid_info.I = 0.0

    def get_pid(self, error, scaler = 1.0):
        error = float(error)
        tnow = time.clock() * 1000
        dt = tnow - self._last_t

        if self._last_t == 0 or dt > 1000:
            dt = 0

            # if this PID hasn't been used for a full second then zero
            # the intergator term. This prevents I buildup from a
            # previous fight mode from causing a massive return before
            # the integrator gets a chance to correct itself
            self.reset_i()

        self._last_t = tnow
        delta_time = float(dt) / 1000.0

        # Compute proportional component
        self._pid_info.P = error * self._Kp;
        output = self._pid_info.P

        # Compute derivative component if time has elapsed
        if abs(self._Kd) > 0 and dt > 0:
            derivative = 0.0

            if math.isnan(self._last_derivative):
                # we've just done a reset, suppress the first derivative
                # term as we don't want a sudden change in input to cause
                # a large D output change
                derivative = 0.0
                self._last_derivative = 0.0
            else:
                derivative = (error - self._last_error) / delta_time

            # discrete low pass filter, cuts out the
            # high frequency noise that can drive the controller crazy
            derivative = self._last_derivative + (delta_time / (RC + delta_time)) * (derivative - self._last_derivative)

            # update state
            self._last_error = error
            self._last_derivative = derivative

            # add in derivative component
            self._pid_info.D = self._Kd * derivative
            output += self._pid_info.D

        # scale the P and D components
        output *= scaler
        self._pid_info.D *= scaler
        self._pid_info.P *= scaler

        # Compute integral component if time has elapsed
        if abs(self._Ki) > 0 and dt > 0:
            self._integrator += (error * self._Ki) * scaler * delta_time

            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax

            self._pid_info.I = self._integrator
            output += self._integrator

        self._pid_info.desired = output

        return output


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
        self.throttle_pid = PID(0.7, 0.2, 0.8)
        self.pid_throttle = 0.0
        self.frame = None
        self.throttle = 0.0
        self.angle = 0.0
        self.brake = False
        self.speed = 0.0
        self.on = True
        self.count = 0
        self.imu = None
        self.epoch = datetime(1970, 1, 1)
        self.kestrel_client = kestrel.Client(subscribers.servers)
        self.controller_subscriber = subscribers.ControllerSubscriberThread(args = (self, "controller"))
        self.image_subscriber = subscribers.ImageSubscriberThread(args = (self, "cam-image.pilot", None, True), name = "PilotImage")
        self.speed_subscriber = subscribers.SpeedSubscriberThread(args = (self, "teensy-speed.pilot"))
        self.imu_subscriber = subscribers.IMUSubscriberThread(args = (self, "astar-imu.pilot"))
        self.controller_subscriber.start()
        self.image_subscriber.start()
        self.speed_subscriber.start()
        self.imu_subscriber.start()

    def constrain(self, v, nmin, nmax):
        return max(min(nmax, v), nmin)

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

    def run(self):
        angle = self.angle
        speed = self.speed
        throttle = self.throttle

        if PilotHandler.active_pilot == None:
            self.throttle_pid.reset_i()
            self.pid_throttle = self.throttle
        else:
            if self.brake:
                self.throttle_pid.reset_i()
                throttle = 0.0
                angle = 0.0
                speed = 0.0
            else:
                with PilotHandler.active_pilot.graph.as_default():
                    angle, throttle, speed =  PilotHandler.active_pilot.decide(self.frame)

                fixed_target_speed = 2.2

                e = fixed_target_speed - self.speed
                self.pid_throttle += self.throttle_pid.get_pid(e, 5.0)
                self.pid_throttle = self.constrain(self.pid_throttle, -500.0, 500.0)
                throttle = self.pid_throttle / 1000.
                speed = fixed_target_speed

                if self.count % 10 == 0:
                    print("pilot values= throttle= %f  angle= %f  speed= %f  odo= %f" %
                        (throttle, angle, speed, self.speed))

                self.count += 1

        d = {}
        d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
        d['steering'] = angle
        d['throttle'] = throttle
        d['speed'] = speed
        self.kestrel_client.add('pilot', json.dumps(d))

    def shutdown(self):
        # indicate that the thread should be stopped
        print('stopping PilotHandler')
        self.on = False;
        self.kestrel_client.close()
        time.sleep(1)
