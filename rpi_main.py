from RPi import GPIO as io
from simple_pid import PID
from time import sleep, time_ns
from numpy import linspace

io.setmode(io.BCM)
io.setwarnings(False)

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
        io.setup(dir, io.OUT)
        io.setup(pwm, io.OUT)
        self._pwm = io.PWM(pwm, 1000)
        self.pwm = lambda x: self._pwm.changeDutyCycle(x)
        self.dir = lambda x: io.output(dir, x)
        self.current_speed = 0
        self.pid_params = (p, i, d)
        self.b = self.brake

    def set(self, speed):
        self.pwm(abs(speed))
        self.dir(speed > 0)
        self.current_speed = abs(speed)

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
        self.set(-self.current_speed)
        sleep(wait)
        self.set(0)

    def reset(self):
        self.brake()
        sleep(0.1)
        self.encoder.reset()


class Encoder:
    def __init__(self, a, b):
        io.add_event_detect(a, io.RISING, callback=self._update)
        io.setup(b, io.IN)
        self.a = a
        self.b = b
        self.ticks = 0
        self.lastTick = 0
        self.speed = 0

    def _update(self, pin):
        if self.lastTick == 0:
            self.lastTick = time_ns()
        else:
            elapsed = time_ns()-self.lastTick
            self.lastTick = time_ns()
            self.speed = 1_000_000/elapsed
        if io.input(self.b):
            self.ticks += 1
        else:
            self.ticks -= 1

    def reset(self):
        io.add_event_detect(self.a, io.RISING, callback=self._update)
        sleep(0.5)
        self.ticks = self.lastTick = self.speed = 0


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
