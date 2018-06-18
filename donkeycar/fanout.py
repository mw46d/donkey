import donkeycar.subscribers as subscribers
import kestrel

c = kestrel.Client(subscribers.servers)

c.fanout("pilot+pilot.steering,pilot.throttle")
c.fanout("cam-image+cam-image.controller,cam-image.pilot")
c.fanout("mode+mode.steering,mode.throttle")
c.fanout("teensy-speed+teensy-speed.pilot")
c.fanout("astar-imu+astar-imu.pilot")
