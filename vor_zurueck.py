from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

motor_links = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_rechts = Motor(Port.F)

METER = 2046
SPEED = 500


def fahren(grad):
    motor_links.run_angle(SPEED, grad, wait=False)
    motor_rechts.run_angle(SPEED, grad)


def knopf_los():
    while Button.CENTER in hub.buttons.pressed():
        wait(10)
    wait(30)


while True:
    while Button.CENTER not in hub.buttons.pressed():
        wait(10)
    knopf_los()
    klicks = 1
    uhr = StopWatch()
    while uhr.time() < 500:
        if Button.CENTER in hub.buttons.pressed():
            klicks += 1
            knopf_los()
            uhr.reset()
        wait(10)
    if klicks == 1:
        fahren(METER)
    elif klicks == 2:
        fahren(-METER)
