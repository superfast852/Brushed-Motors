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
    def __init__(self, a, b):
        self.callback = pi.callback(a, 0, self._update)
        self.a = a
        self.b = b
        self.ticks = 0
        self.lastTick = 0
        self.speed = 0

    def _update(self, gpio, level, tick):
        if self.lastTick == 0:
            self.lastTick = tick
        else:
            elapsed = tick-self.lastTick
            self.lastTick = tick
            self.speed = 1_000_000/elapsed
        if pi.read(self.b):
            self.ticks += 1
        else:
            self.ticks -= 1

    def reset(self):
        self.callback.cancel()
        sleep(0.5)
        self.ticks = self.lastTick = self.speed = 0
        self.callback = pi.callback(self.a, 0, self._update)


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
