import io
import os
import time
from threading import Thread
from itertools import cycle

import numpy as np

from PIL import Image

from donkey import utils

class BaseCamera:
    def __init__(self, resolution=(160, 120)):
        self.resolution = resolution
        self.frame = np.zeros(shape=(self.resolution[1], self.resolution[0], 3))

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        time.sleep(1)
        return self

    def update(self):
        while True:
            pass

    def read(self):
        return self.frame

    def capture_arr(self):
        return self.read()

    def capture_img(self):
        arr = self.capture_arr()
        print(type(arr))
        img = Image.fromarray(arr, 'RGB')
        return img

    def capture_binary(self):
        img = self.capture_img()
        return utils.img_to_binary(img)

class WebcamVideoStream(BaseCamera):
    def __init__(self, resolution = (160, 120), framerate = 20):
        import pygame
        import pygame.camera

        super().__init__(resolution = resolution)

        pygame.init()
        pygame.camera.init()
        l = pygame.camera.list_cameras()
        self.cam = pygame.camera.Camera(l[0], resolution, "RGB")
        self.cam.start()
        self.framerate = framerate

        # initialize variable used to indicate
        # if the thread should be stopped
        self.stopped = False

        print('WebcamVideoStream loaded.. .warming camera')

        time.sleep(2)
        self.start()

    def update(self):
        from datetime import datetime, timedelta
        import pygame.image
        while not self.stopped:
            start = datetime.now()

            if self.cam.query_image():
                # snapshot = self.cam.get_image()
                # self.frame = list(pygame.image.tostring(snapshot, "RGB", False))
                snapshot = self.cam.get_image()
                snapshot1 = pygame.transform.scale(snapshot, self.resolution)
                self.frame = pygame.surfarray.pixels3d(pygame.transform.rotate(pygame.transform.flip(snapshot1, True, False), 90))

            stop = datetime.now()
            s = 1 / self.framerate - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

        self.cam.stop()

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

class PiVideoStream(BaseCamera):
    def __init__(self, resolution=(160, 120), framerate=20):
        from picamera.array import PiRGBArray
        from picamera import PiCamera

        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
            format="rgb", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

        print('PiVideoStream loaded.. .warming camera')

        time.sleep(2)
        self.start()

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return


    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True


class ImgArrayCamera(BaseCamera):
    """
    Used by simulate script.
    """

    def __init__(self, X):
        self.X = X
        self.frame = X[0]
        self.start()


    def generator(self):
        while True:
            for i in self.X:
                yield i

    def update(self):
        # keep looping infinitely until the thread is stopped
        for x in self.generator():
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = x
            time.sleep(.2)

class FakeCamera(BaseCamera):
    '''
    Class that acts like a PiCamera but reads files from a dir.
    Used for testing on non-Pi devices.
    '''
    def __init__(self, img_paths, **kwargs):
        print('loading FakeCamera')

        self.file_list = img_paths
        self.file_list.sort()
        self.file_cycle = cycle(self.file_list) #create infinite iterator
        self.counter = 0

        # if the thread should be stopped
        self.frame = None
        self.start()


    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.file_cycle:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = np.array(Image.open(f))
            self.counter += 1
            time.sleep(.2)

class BaseSpeed:
    def __init__(self):
        self.speed = 0
        self.linaccel = None
        # initialize variable used to indicate
        # if the thread should be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        time.sleep(1)
        return self

    def update(self):
        while True:
            pass

    def read_speed(self):
        return self.speed

    def read_linaccel(self):
        return self.linaccel

class MaestroSpeed(BaseSpeed):
    def __init__(self):
        import donkey as dk
        super().__init__()
        self.sensor = dk.actuators.Maestro_Controller(5);

        self.start()

    def update(self):
        from datetime import datetime, timedelta
        import re

        encoder_pattern = re.compile('^E ([-0-9]+)( ([-0-9]+))?( ([-0-9]+))?$')
        linaccel_pattern = re.compile('^L ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-0-9]+)$')

        while not self.stopped:
            start = datetime.now()

            l = self.sensor.readline()
            while l:
                m = encoder_pattern.match(l.decode('utf-8'))

                if m:
                    value = int(m.group(1))
                    # rospy.loginfo("%s: Receiver E got %d" % (self.node_name, value))
                    # Speed
                    # 40 ticks/wheel rotation,
                    # circumfence 0.377m
                    # every 0.1 seconds
                    if len(m.group(3)) > 0:
                        period = 0.001 * int(m.group(3))
                    else:
                        period = 0.1

                    self.speed = 0.377 * (float(value) / 40) / period   # now in m/s
                else:
                    m = linaccel_pattern.match(l.decode('utf-8'))

                    if m:
                        la = { 'x': float(m.group(1)), 'y': float(m.group(2)), 'z': float(m.group(3)) }

                        self.linaccel = la
                        print("mw linaccel= " + str(self.linaccel))

                l = self.sensor.readline()

            stop = datetime.now()
            s = 0.1 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

