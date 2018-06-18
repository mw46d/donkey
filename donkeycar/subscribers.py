import base64
import json
import kestrel
import SharedArray as sa
import threading

servers = [ '127.0.0.1:22133' ]

class BaseSubscriberThread(threading.Thread):
    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None):
        threading.Thread.__init__(self, group = group, target = target, name = name)
        self.args = args
        self.collector = None
        self.run_part = False
        self.part = args[0]
        self.queue = args[1]

        if len(args) > 2:
            self.collector = args[2]

        if len(args) > 3:
            self.run_part = args[3]

        self.kwargs = kwargs
        return

    def collector_add(self, e):
        if self.collector is not None:
            self.collector[self.queue.split('.')[0]] = e

class ControllerSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 200)
            if e is not None:
                j = json.loads(e)
                self.part.throttle = j['throttle']
                self.part.angle = j['steering']
                self.part.brake = j['brake']

                self.collector_add(e)

        c.close()

class ImageSubscriberThread(BaseSubscriberThread):
    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None):
        super().__init__(group = group, target = target, name = name,
            args = args, kwargs = kwargs)

        self.shared_image_array = sa.attach("shm://camera-image-array")
        self.last_index = -1

    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 100)
            if e is not None:
                while True:
                    e1 = c.get(self.queue)
                    if e1 is None:
                        break
                    else:
                        e = e1

                j = json.loads(e)

                image_index = j['image_index']
                self.part.frame = self.shared_image_array[image_index % 8]

                if self.last_index >= 0 and self.last_index < image_index - 1:
                    print("ImageSubscriberThread(%s) lost %d frames!!" % (threading.current_thread().name, image_index - 1 - self.last_index))

                self.last_index = image_index
                self.collector_add(e)

                if self.run_part:
                    self.part.run()

        c.close()


class ModeSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 2000)
            if e is not None:
                j = json.loads(e)
                self.part.mode = j['mode']

                self.collector_add(e)

        c.close()

class PilotSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            # print("PilotSubscriberThread loop start")
            e = c.get(self.queue, timeout = 200)
            if e is not None:
                while True:
                    e1 = c.get(self.queue)
                    if e1 is None:
                        break
                    else:
                        e = e1
                # while c.peek(self.queue) is not None:
                #     e = c.get(self.queue)

                j = json.loads(e)
                self.part.throttle = j['throttle']
                self.part.angle = j['steering']
                self.part.target_speed = j['speed']

                self.collector_add(e)
            else:
                self.part.throttle = 0.0
                self.part.angle = 0.0
                self.part.target_speed = 0.0

            # print("PilotSubscriberThread loop %s %f %f %f" % (self.queue, self.controller.throttle, self.controller.angle, self.controller.angle))
            if self.run_part:
                self.part.run()

        c.close()

class RCinSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 200)
            if e is not None:
                j = json.loads(e)
                self.part.angle = j['steering']
                self.part.throttle = j['throttle']

                self.collector_add(e)
            else:
                self.part.angle = 0.0
                self.part.throttle = 0.0

        c.close()

class IMUSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 200)
            if e is not None:
                j = json.loads(e)
                self.part.imu = j['imu']

                self.collector_add(e)

        c.close()

class RecordingSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 2000)
            if e is not None:
                j = json.loads(e)
                self.part.recording = j['recording']

                self.collector_add(e)

        c.close()

class SpeedSubscriberThread(BaseSubscriberThread):
    def run(self):
        c = kestrel.Client(servers)

        while self.part.on:
            e = c.get(self.queue, timeout = 200)
            if e is not None:
                j = json.loads(e)
                self.part.speed = j['speed']

                self.collector_add(e)

        c.close()

