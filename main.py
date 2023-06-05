import pigpio
from simple_pid import PID

pi = pigpio.pi()


def in_tolerance(a, b, tolerance=10):
    return (b-tolerance) < a > (b+tolerance)


class Motor:
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

    def setPosition(self, ticks):
        p, i, d = self.pid_params
        pid = PID(p, i, d, ticks, output_limits=(-1, 1), sample_time=0.05)
        out = []
        while not in_tolerance(self.encoder.ticks, ticks):
            v = pid(self.encoder.ticks)
            self.set(v)
            out.append(v)
        return out

    def goTo(self, ticks, speed=0.25, tolerance=25):
        if ticks > self.encoder.ticks:
            direction = 1
        elif ticks < self.encoder.ticks:
            direction = -1
        lastDistance = abs(ticks - self.encoder.ticks)
        while not in_tolerance(self.encoder.ticks, ticks, tolerance):
            print(self.encoder.ticks)
            self.set(speed)
            distance = abs(ticks - self.encoder.ticks)
            if distance > lastDistance:
                direction = not direction
            lastDistance = distance
        self.brake()
        return abs(ticks-self.encoder.ticks)

    def brake(self):
        self.set(0)


class Encoder:
    def __init__(self, a, b):
        self.callback = pi.callback(a, 0, self._update)
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


if __name__ == "__main__":
    from math import sin, radians
    motor = Motor(13, 6, 17, 27, 0.5, 0.5, 1)
    i = 0
    divider = 10
    try:
        while True:
            speed = sin(radians(i/divider))
            motor.set(speed)
            print(motor.encoder.ticks, motor.encoder.speed, speed)
            i += 1
    except KeyboardInterrupt:
        motor.set(0)
        motor.encoder.callback.cancel()
