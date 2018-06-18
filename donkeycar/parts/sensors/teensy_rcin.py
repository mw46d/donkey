from datetime import datetime, timedelta
import donkeycar as dk
import donkeycar.subscribers as subscribers
import donkeycar.utils as utils
import json
import kestrel
import re
import signal
import time
from threading import Thread

class TeensyRCin:
    def __init__(self):
        self.inSteering = 0.0
        self.inThrottle = 0.0
        self.speed = 0.0

        self.sensor = dk.parts.Teensy(0);

        TeensyRCin.LEFT_ANGLE = -1.0
        TeensyRCin.RIGHT_ANGLE = 1.0
        TeensyRCin.MIN_THROTTLE = -1.0
        TeensyRCin.MAX_THROTTLE =  1.0

        TeensyRCin.LEFT_PULSE = 496.0
        TeensyRCin.RIGHT_PULSE = 242.0
        TeensyRCin.MAX_PULSE = 496.0
        TeensyRCin.MIN_PULSE = 242.0

        self.epoch = datetime(1970, 1, 1)
        self.on = True

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        '''
        Linear mapping between two ranges of values
        '''
        X_range = X_max - X_min
        Y_range = Y_max - Y_min
        XY_ratio = X_range/Y_range

        return ((x-X_min) / XY_ratio + Y_min)

    def update(self):
        rcin_pattern = re.compile('^I +([.0-9]+) +([.0-9]+).*$')
        encoder_pattern = re.compile('^E ([-0-9]+)( ([-0-9]+))?( ([-0-9]+))?$')

        while self.on:
            start = datetime.now()

            l = self.sensor.teensy_readline()
            c = kestrel.Client(subscribers.servers)

            while l:
                # print("mw TeensyRCin line= " + l.decode('utf-8'))
                m = rcin_pattern.match(l.decode('utf-8'))

                if m:
                    i = float(m.group(1))
                    if i == 0.0:
                        self.inSteering = 0.0
                    else:
                        i = i / (1000.0 * 1000.0) # in seconds
                        i *= self.sensor.frequency * 4096.0
                        self.inSteering = self.map_range(i,
                                                         TeensyRCin.LEFT_PULSE, TeensyRCin.RIGHT_PULSE,
                                                         TeensyRCin.LEFT_ANGLE, TeensyRCin.RIGHT_ANGLE)

                    k = float(m.group(2))
                    if k == 0.0:
                        self.inThrottle = 0.0
                    else:
                        k = k / (1000.0 * 1000.0) # in seconds
                        k *= self.sensor.frequency * 4096.0
                        self.inThrottle = self.map_range(k,
                                                         TeensyRCin.MIN_PULSE, TeensyRCin.MAX_PULSE,
                                                         TeensyRCin.MIN_THROTTLE, TeensyRCin.MAX_THROTTLE)

                    # print("matched %.1f  %.1f  %.1f  %.1f" % (i, self.inSteering, k, self.inThrottle))

                    d = {}
                    d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
                    d['steering'] = self.inSteering
                    d['throttle'] = self.inThrottle
                    c.add('teensy-rcin', json.dumps(d))
                else:
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

                        d = {}
                        d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
                        d['speed'] = self.speed
                        c.add('teensy-speed', json.dumps(d))

                l = self.sensor.teensy_readline()

            c.close()
            stop = datetime.now()
            s = 0.01 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        return self.inSteering, self.inThrottle

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping TeensyRCin')
        time.sleep(.5)

def main():
    teensy = TeensyRCin()
    t = Thread(target = teensy.update, args=())
    t.start()

    with utils.GracefulInterruptHandler(sig = signal.SIGINT) as h1:
        with utils.GracefulInterruptHandler(sig = signal.SIGTERM) as h2:
            while True:
                if h1.interrupted:
                    break
                if h2.interrupted:
                    break

                time.sleep(5)

    teensy.shutdown()

if __name__ == '__main__':
    main()

