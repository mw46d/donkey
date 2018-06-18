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

class AStarSpeed:
    def __init__(self):
        self.speed = 0.0
        self.imu = None

        self.sensor = dk.parts.Teensy(0);

        self.epoch = datetime(1970, 1, 1)
        self.on = True

    def update(self):
        encoder_pattern = re.compile('^E ([-0-9]+)( ([-0-9]+))?( ([-0-9]+))?$')
        imu_pattern = re.compile('^A ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-0-9]+) ([-0-9]+)$')
        linaccel_pattern = re.compile('^L ([-.0-9]+) ([-.0-9]+) ([-.0-9]+) ([-0-9]+)$')

        while self.on:
            start = datetime.now()

            l = self.sensor.astar_readline()
            c = kestrel.Client(subscribers.servers)

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

                    d = {}
                    d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
                    d['speed'] = self.speed
                    c.add('astar-speed', json.dumps(d))
                else:
                    m = imu_pattern.match(l.decode('utf-8'))

                    if m:
                        a = {
                            'acl_x': float(m.group(1)),
                            'acl_y': float(m.group(2)),
                            'acl_z': float(m.group(3)),
                            'gyr_x': float(m.group(4)),
                            'gyr_x': float(m.group(5)),
                            'gyr_x': float(m.group(6)),
                            'temp': int(m.group(7))
                        }

                        self.imu = a
                        # print("mw accel= " + str(self.accel))

                        d = {}
                        d['timestamp'] = (datetime.utcnow() - self.epoch).total_seconds()
                        d['imu'] = self.imu
                        c.add('astar-imu', json.dumps(d))

                l = self.sensor.astar_readline()


            c.close()
            stop = datetime.now()
            s = 0.01 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        return self.speed, self.accel, self.gyro

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping AStarSpeed')
        time.sleep(.5)

def main():
    astar = AStarSpeed()
    t = Thread(target = astar.update, args=())
    t.start()

    with utils.GracefulInterruptHandler(sig = signal.SIGINT) as h1:
        with utils.GracefulInterruptHandler(sig = signal.SIGTERM) as h2:
            while True:
                if h1.interrupted:
                    break
                if h2.interrupted:
                    break

                time.sleep(5)

    astar.shutdown()

if __name__ == '__main__':
    main()

