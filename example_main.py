import pigpio
from simple_pid import PID
from time import sleep
from numpy import linspace

pi = pigpio.pi()


def in_tolerance(a, b, tolerance=10):
    return (b-tolerance) < a < (b+tolerance)


def smoothSpeed(x, target, steps=100):
    height = abs(x-target)
    if height == 0:
        return [0.0, 0.0]
    slope = 10/height if target < x else -10/height
    yshift = min(x, target)

    return [(height / (1 + 2.71828**(slope*i+5))+yshift)
            for i in linspace(0 if x < target else -height, height if x < target else 0, steps)]


class Motor:  # TODO: Add Sigmoid tick approach and angle-tick conversion
    def __init__(self, pwm, dir, a, b, p, i, d):
        self.encoder = Encoder(a, b)
        self.speed = 0
        pi.set_mode(dir, pigpio.OUTPUT)
        pi.set_PWM_range(pwm, 100)
        self._pwm = pwm
        self.dir = dir
        self.pid_params = (p, i, d)
        self.b = self.brake

    def set(self, speed):
        pi.set_PWM_dutycycle(self._pwm, abs(speed*100))
        pi.write(self.dir, speed > 0)

    def setPosition(self, ticks):  # TODO: This brokey. Need to Fix.
        p, i, d = self.pid_params
        pid = PID(p, i, d, ticks, output_limits=(-1, 1), sample_time=0.05)
        out = []
        while not in_tolerance(self.encoder.ticks, ticks):
            v = pid(self.encoder.ticks)
            self.set(v)
            out.append(v)
        return out

    def goTo(self, ticks, speed=0.25, tolerance=5, direction_det=25):  # Rebuild this. lastDistance Shouldn't be negative.
        if ticks > self.encoder.ticks:
            direction = -1
        elif ticks < self.encoder.ticks:
            direction = 1
        else:
            direction = 0
            self.encoder.ticks = ticks
            return 0
        lastDistance = abs(ticks - self.encoder.ticks)
        while not in_tolerance(self.encoder.ticks, ticks, tolerance):
            self.set(speed*direction)
            distance = abs(ticks - self.encoder.ticks)
            print("Ticks: {}; Distance: {}; Distance Comparison: {}; Speed Applied: {}; Encoder Speed: {}".format(
                self.encoder.ticks, distance, lastDistance+direction_det, speed*direction, self.encoder.speed))
            if distance > lastDistance+direction_det:
                direction = -direction
                self.set(speed*direction)
                sleep(0.1)
                distance = lastDistance-direction_det
            lastDistance = distance if distance < lastDistance else lastDistance
        self.brake()
        return abs(ticks - self.encoder.ticks)

    def brake(self, wait=0.05):
        pi.write(self.dir, not pi.read(self.dir))
        sleep(wait)
        self.set(0)

    def reset(self):
        self.brake()
        sleep(0.5)
        self.encoder.reset()


class Encoder:

    """Class to decode mechanical rotary encoder pulses."""

    def __init__(self, gpioA, gpioB):

        """
      Instantiate the class with the pi and gpios connected to
      rotary encoder contacts A and B.  The common contact
      should be connected to ground.  The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for clockwise and -1 for
      counterclockwise.

      EXAMPLE

      import time
      import pigpio

      import rotary_encoder

      pos = 0

      def callback(way):

         global pos

         pos += way

         print("pos={}".format(pos))

      pi = pigpio.pi()

      decoder = rotary_encoder.decoder(pi, 7, 8, callback)

      time.sleep(300)

      decoder.cancel()

      pi.stop()

      """
        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)
        self.ticks = 0
        self.speed = 0
        self.lastTick = 0

    def _pulse(self, gpio, level, tick):

        """
        Decode the rotary encoder pulse.

                     +---------+         +---------+      0
                     |         |         |         |
           A         |         |         |         |
                     |         |         |         |
           +---------+         +---------+         +----- 1

               +---------+         +---------+            0
               |         |         |         |
           B   |         |         |         |
               |         |         |         |
           ----+         +---------+         +---------+  1
        """

        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.ticks += 1
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.ticks -= 1
        if self.lastTick == 0:
            self.lastTick = tick
        else:
            self.speed = 1_000_000/(tick-self.lastTick)
            self.lastTick = tick

    def reset(self):
        """
        Resets the decoder to zero.
        """
        self.ticks = 0
        self.cbA.cancel()
        self.cbB.cancel()

    def exit(self):
        self.cbA.cancel()
        self.cbB.cancel()


if __name__ == "__main__":
    from math import sin, radians
    mtr = Motor(13, 6, 17, 27, 0.5, 0.5, 1)
    i = 0
    divider = 10
    try:
        while True:
            speed = sin(radians(i/divider))
            mtr.set(speed)
            print(mtr.encoder.ticks, mtr.encoder.speed, speed)
            i += 1
    except KeyboardInterrupt:
        mtr.set(0)
        mtr.encoder.callback.cancel()
