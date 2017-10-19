from datetime import datetime, timedelta
import donkeycar as dk
import re
import time

class TeensySpeed:
    def __init__(self):
        self.speed = 0.0
        self.linaccel = None

        self.sensor = dk.parts.Teensy(0);

        self.on = True

    def update(self):
        encoder_pattern = re.compile('^E ([-0-9]+)( ([-0-9]+))?( ([-0-9]+))?$')

        while self.on:
            start = datetime.now()

            l = self.sensor.teensy_readline()
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

                l = self.sensor.teensy_readline()

            stop = datetime.now()
            s = 0.1 - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

    def run_threaded(self):
        return self.speed

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stopping TeensySpeed')
        time.sleep(.5)

