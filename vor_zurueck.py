from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

motor_links = Motor(Port.C, Direction.COUNTERCLOCKWISE)
motor_rechts = Motor(Port.E)

RAD_DURCHMESSER = 56
SPURBREITE = 112
STRECKE = 1000
DOPPELKLICK_ZEIT = 500

fahrwerk = DriveBase(motor_links, motor_rechts, RAD_DURCHMESSER, SPURBREITE)


def warten_bis_losgelassen():
    while Button.CENTER in hub.buttons.pressed():
        wait(10)
    wait(30)


def klicks_zaehlen():
    while Button.CENTER not in hub.buttons.pressed():
        wait(10)
    warten_bis_losgelassen()
    klicks = 1
    uhr = StopWatch()
    while uhr.time() < DOPPELKLICK_ZEIT:
        if Button.CENTER in hub.buttons.pressed():
            klicks += 1
            warten_bis_losgelassen()
            uhr.reset()
        wait(10)
    return klicks


while True:
    klicks = klicks_zaehlen()
    if klicks == 1:
        fahrwerk.straight(STRECKE)
    elif klicks == 2:
        fahrwerk.straight(-STRECKE)
