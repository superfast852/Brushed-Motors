from main import Motor, sleep
motor = Motor(13, 6, 17, 27, 0.5, 0.5, 1)


def quickTest():
    motor.set(1)
    sleep(1)
    motor.b()
    sleep(1)
    print(motor.encoder.ticks)
    motor.encoder.ticks = 0

    motor.set(-1)
    sleep(1)
    motor.b()
    sleep(1)
    print(motor.encoder.ticks)
    motor.encoder.ticks = 0


def monitor(speed):
    try:
        while 1:
            motor.set(speed)
            print(motor.encoder.ticks, motor.encoder.speed)

    except KeyboardInterrupt:
        motor.reset()
