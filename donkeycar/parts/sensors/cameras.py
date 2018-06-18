#!/usr/bin/python

import base64
import cv2
from datetime import datetime, timedelta
import donkeycar.subscribers as subscribers
import donkeycar.utils as utils
import json
import kestrel
import imutils
import numpy as np
import SharedArray as sa
import signal
import sys
import time
from threading import Thread

class BaseCamera:

    def run_threaded(self):
        return self.frame

class PiCamera(BaseCamera):
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
        self.on = True

        print('PiCamera loaded.. .warming camera')
        time.sleep(2)


    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            if not self.on:
                break

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

class Webcam(BaseCamera):
    def __init__(self, resolution = (160, 120), framerate = 20):
        super().__init__()

        self.frame = None

        self.cam = cv2.VideoCapture(0);
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cam.set(cv2.CAP_PROP_FPS, 120)
        self.resolution = resolution
        self.framerate = framerate
        self.epoch = datetime(1970, 1, 1)
        # self.input_window = cv2.namedWindow("Input")
        # self.output_window = cv2.namedWindow("Output")

        try:
	        sa.delete("shm://camera-image-array")
        except:
	        pass

        self.shared_image_array = sa.create("shm://camera-image-array", (8, 120, 160, 3), np.uint8)
        self.shared_index = 0

        # initialize variable used to indicate
        # if the thread should be stopped
        self.on = True

        print('WebcamVideoStream loaded.. .warming camera')

        time.sleep(3)

    def update(self):
        c = kestrel.Client(subscribers.servers)
        t_start = 0
        t_count = 0

        while self.on:
            start = datetime.now()

            rval, snapshot = self.cam.read()
            snapshot = cv2.resize(snapshot, self.resolution, interpolation = cv2.INTER_AREA)
            # snapshot = cv2.flip(snapshot, 1)
            # snapshot = cv2.rotate(snapshot, 90)

            # cv2.imshow('Input', snapshot)
            key = cv2.waitKey(1)

            # print("mw t1 %s" % str(snapshot))
            t1 = time.time()

            if t_count == 0:
                print ("time %6.4f" % (float(t1 - t_start) / 100.0))
                t_start = t1

            t_count = (t_count + 1) % 100

            self.shared_image_array[self.shared_index % 8] = snapshot
            sa.msync(self.shared_image_array, sa.MS_SYNC | sa.MS_INVALIDATE)
            self.frame = snapshot

            d = {}
            d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
            d['image_index'] = self.shared_index
            d['resolution'] = self.resolution
            c.add('cam-image', json.dumps(d))

            f = self.shared_image_array[self.shared_index % 8]
            # cv2.imshow('Output', f)
            self.shared_index += 1

            stop = datetime.now()
            s = 1.0 / self.framerate - (stop - start).total_seconds() - 0.0002
            if s > 0.0:
                time.sleep(s)

        self.cam.stop()
        c.close()

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping Webcam')
        time.sleep(.5)

def main():
    cam = Webcam(framerate = 30)
    t = Thread(target = cam.update, args=())
    t.start()

    with utils.GracefulInterruptHandler(sig = signal.SIGINT) as h1:
        with utils.GracefulInterruptHandler(sig = signal.SIGTERM) as h2:
            while True:
                if h1.interrupted:
                    break
                if h2.interrupted:
                    break

                time.sleep(5)

    cam.shutdown()

if __name__ == '__main__':
    main()

