from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

motor_links = Motor(Port.C, Direction.COUNTERCLOCKWISE)
motor_rechts = Motor(Port.E)
sensor_links = ColorSensor(Port.B)
sensor_rechts = ColorSensor(Port.D)

SCHWARZ = 10
GRENZE = 0.35
TIEF = 0.2
MAX_SPEED = 800
TURBO_SPEED = 1000
ANLAUF = 0.6
BREMSE = 0.4
LENK_START = 0.7
LENK_ZUWACHS = 3.0
KP_KURS = 5
KURS_RICHTUNG = 1
ENTPRELL = 6
MITTEL = 3
MIN_INNEN = 0.0
SCHLITZ_SPEED = 0.85

for motor in (motor_links, motor_rechts):
    try:
        motor.control.limits(acceleration=8000)
    except Exception:
        pass

while not hub.imu.ready():
    wait(10)
hub.imu.reset_heading(0)

wait(300)


def kalibrieren(sensor):
    summe = 0
    for _ in range(20):
        summe += sensor.reflection()
        wait(5)
    return max(summe / 20, SCHWARZ + 20)


weiss_links = kalibrieren(sensor_links)
weiss_rechts = kalibrieren(sensor_rechts)


class Seite:
    def __init__(self, sensor, weiss):
        self.sensor = sensor
        self.weiss = weiss
        self.werte = [weiss] * MITTEL
        self.index = 0
        self.uhr = StopWatch()
        self.dunkel_aktiv = False
        self.hell = 1
        self.roh = 0
        self.stark = 0

    def messen(self):
        self.werte[self.index] = self.sensor.reflection()
        self.index = (self.index + 1) % MITTEL
        mittel = sum(self.werte) / MITTEL
        hell = (mittel - SCHWARZ) / (self.weiss - SCHWARZ)
        hell = max(0, min(1, hell))
        self.hell = hell
        self.roh = 0 if hell >= GRENZE else (GRENZE - hell) / GRENZE

        if self.roh > 0:
            if not self.dunkel_aktiv:
                self.uhr.reset()
                self.dunkel_aktiv = True
            self.stark = self.roh if self.uhr.time() >= ENTPRELL else 0
        else:
            self.dunkel_aktiv = False
            self.stark = 0


links = Seite(sensor_links, weiss_links)
rechts = Seite(sensor_rechts, weiss_rechts)

ziel_kurs = 0
richtung = 0
war_schwarz = False
schwarz_uhr = StopWatch()
weiss_uhr = StopWatch()

while True:
    links.messen()
    rechts.messen()
    dl = links.stark
    dr = rechts.stark
    if dl > 0 and dr > 0 and links.hell > TIEF and rechts.hell > TIEF:
        dl = 0
        dr = 0
    kurs = KURS_RICHTUNG * hub.imu.heading()

    if dl > 0 and dr == 0:
        richtung = 1
    elif dr > 0 and dl == 0:
        richtung = -1
    elif dl > 0 and dr > 0:
        if richtung == 0:
            richtung = 1 if dl >= dr else -1
    else:
        richtung = 0

    if richtung != 0:
        if not war_schwarz:
            schwarz_uhr.reset()
            war_schwarz = True

        staerke = 1 if (dl > 0 and dr > 0) else max(dl, dr)
        lenk = staerke * LENK_START + schwarz_uhr.time() / 1000 * LENK_ZUWACHS
        lenk = min(1, lenk)

        speed = MAX_SPEED * (1 - BREMSE * lenk)
        aussen = speed
        innen = max(MIN_INNEN, speed * (1 - lenk))

        if richtung == 1:
            v_links, v_rechts = aussen, innen
        else:
            v_links, v_rechts = innen, aussen
    else:
        if war_schwarz:
            ziel_kurs = kurs
            war_schwarz = False
            weiss_uhr.reset()

        anteil = min(1, weiss_uhr.time() / 1000 / ANLAUF)
        speed = MAX_SPEED + (TURBO_SPEED - MAX_SPEED) * anteil

        if links.roh > 0 or rechts.roh > 0:
            speed *= SCHLITZ_SPEED

        korrektur = KP_KURS * (ziel_kurs - kurs)
        korrektur = max(-speed * 0.3, min(speed * 0.3, korrektur))
        v_links = max(0, min(speed, speed + korrektur))
        v_rechts = max(0, min(speed, speed - korrektur))

    motor_links.run(v_links)
    motor_rechts.run(v_rechts)
    wait(1)
