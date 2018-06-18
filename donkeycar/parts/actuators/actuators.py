#!/usr/bin/python
"""
actuators.py

Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.

"""

import donkeycar.subscribers as subscribers
import donkeycar.utils as utils
import signal
import time

class PCA9685:
    '''
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    '''
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)

class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout = 0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout = 0.01)

    def set_speed(self, speed):
        print("Teensy::set_speed= %f" % speed)
        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("V %.2f\n" % speed).encode('ascii'))

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=290,
                       right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.throttle = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.mode = None
        self.on = True

        self.pilot_subscriber = subscribers.PilotSubscriberThread(args = (self, "pilot.steering", None, True), name = "PWMSteering::PilotSubscriber")
        self.mode_subscriber = subscribers.ModeSubscriberThread(args = (self, "mode.steering"), name = "PWMSteering::ModeSubscriber")
        self.pilot_subscriber.start()
        self.mode_subscriber.start()

    def run(self):
        pulse = utils.map_range(self.angle,
                                self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                self.left_pulse, self.right_pulse)

        if self.mode != None and self.mode != 'user':
            self.controller.set_pulse(pulse)

        self.controller.set_turn_left(self.angle < -0.2)
        self.controller.set_turn_right(self.angle > 0.2)

    def shutdown(self):
        print("PWMSteering shutdown started")
        self.on = False
        self.angle = 0.0
        self.run()
        print("PWMSteering shutdown done")


class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=300,
                       min_pulse=490,
                       zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.throttle = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.mode = None
        self.on = True

        #send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)

        self.pilot_subscriber = subscribers.PilotSubscriberThread(args = (self, "pilot.throttle", None, True), name = "PWMThorttle::PilotSubscriber")
        self.mode_subscriber = subscribers.ModeSubscriberThread(args = (self, "mode.throttle"), name = "PWMThrottle::ModeSubscriber")
        self.pilot_subscriber.start()
        self.mode_subscriber.start()

    def run(self):
        if self.throttle > 0:
            pulse = utils.map_range(self.throttle,
                                    0, self.MAX_THROTTLE,
                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = utils.map_range(self.throttle,
                                    self.MIN_THROTTLE, 0,
                                    self.min_pulse, self.zero_pulse)

        if self.mode != None and self.mode != 'user':
            if self.speed != 0:
                self.controller.set_speed(self.speed)
            else:
                self.controller.set_pulse(pulse)

        self.controller.set_brake(self.throttle < 0)
        self.controller.set_headlight(self.throttle < -0.02 or self.throttle > 0.02)

    def shutdown(self):
        print("PWMThrottle shutdown started")
        self.on = False
        self.throttle = 0.0
        self.speed = 0.0
        self.run()
        print("PWMThrottle shutdown done")

class Adafruit_DCMotor_Hat:
    '''
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0


    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(utils.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)

        self.motor.setSpeed(self.throttle)


    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


def main():
    steering_controller = Teensy('S')
    steering = PWMSteering(controller = steering_controller,
                           left_pulse = 496, right_pulse = 242)

    throttle_controller = Teensy('T')
    throttle = PWMThrottle(controller = throttle_controller,
                           max_pulse = 496, zero_pulse = 369, min_pulse = 242)

    with utils.GracefulInterruptHandler(sig = signal.SIGINT) as h1:
        with utils.GracefulInterruptHandler(sig = signal.SIGTERM) as h2:
            while True:
                if h1.interrupted:
                    break
                if h2.interrupted:
                    break

                print("Endless loop")
                time.sleep(5)

    steering.shutdown()
    throttle.shutdown()

if __name__ == '__main__':
    main()
