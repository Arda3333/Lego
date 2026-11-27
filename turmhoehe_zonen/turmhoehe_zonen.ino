#include <Wire.h>
#include <SPI.h>
#include <Adafruit_VL53L1X.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   4
#define DATA_PIN      23
#define CS_PIN        5
#define CLK_PIN       18
#define HELLIGKEIT    4
#define TASTE_PIN     0

#define SDA_PIN          21
#define SCL_PIN          22
#define ROI_GROESSE      4
#define ZONE_TIMING_MS   20
#define SIGNAL_SCHWELLE  300
#define SIGMA_SCHWELLE   30

#define NPOS             4
#define NZ               16
const int POS[NPOS] = {3, 6, 9, 12};

#define BLOCK_MM         37
#define MAX_BLOECKE      17
#define EIN_MM           10.0f
#define AUS_MM           6.0f
#define DUNKEL_SIGNAL    0.8f
#define VOLLSCAN_ALLE    4
#define STABIL_NOETIG    2
#define HYSTERESE        0.15f

#define LEER 0,
MD_MAX72XX::fontType_t zahlenFont[] PROGMEM = {
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER
  1, 0x40,
  LEER
  4, 0x3E, 0x41, 0x41, 0x3E,
  3, 0x42, 0x7F, 0x40,
  4, 0x62, 0x51, 0x49, 0x46,
  4, 0x22, 0x41, 0x49, 0x36,
  4, 0x18, 0x14, 0x12, 0x7F,
  4, 0x27, 0x45, 0x45, 0x39,
  4, 0x3E, 0x49, 0x49, 0x32,
  4, 0x01, 0x71, 0x09, 0x07,
  4, 0x36, 0x49, 0x49, 0x36,
  4, 0x26, 0x49, 0x49, 0x3E,
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER
  LEER
  3, 0x38, 0x44, 0x44,
  LEER LEER LEER LEER LEER LEER LEER LEER LEER
  5, 0x7C, 0x04, 0x78, 0x04, 0x78,
};

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

float baseD[NZ];
float baseS[NZ];
bool  baseOK[NZ];
float zoneH[NZ];
bool  aktiv[NZ];
float zoneD[NZ];
float zoneRatio[NZ];

int   bloecke = 0;
int   letzterKandidat = -1;
int   stabil = 0;
int   durchgang = 0;
bool  debug = false;
bool  rohModus = false;
bool  tasteAlt = HIGH;

void zeige(const char *text) {
  display.setFont(nullptr);
  display.displayClear();
  display.print(text);
}

void zeigeText(const char *text) {
  display.setFont(zahlenFont);
  display.setCharSpacing(1);
  display.displayClear();
  display.print(text);
}

void zeigeBloecke(int b) {
  int mm = b * BLOCK_MM;
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", mm / 10, mm % 10);
  if (!rohModus) zeigeText(text);
  Serial.printf(">> %d Bloecke = %s\n", b, text);
}

uint8_t spadNummer(int col, int row) {
  if (row < 8) return 128 + 8 * col + row;
  return 127 - 8 * col - (row - 8);
}

bool messeZone(int z, int nSamples, float &dist, float &sig) {
  sensor.stopRanging();
  sensor.VL53L1X_SetROI(ROI_GROESSE, ROI_GROESSE);
  sensor.VL53L1X_SetROICenter(spadNummer(POS[z % NPOS], POS[z / NPOS]));
  sensor.startRanging();
  int verworfen = 0, gelesen = 0, gut = 0;
  float summeD = 0, summeS = 0;
  unsigned long start = millis();
  while (gelesen < nSamples && millis() - start < 400) {
    if (sensor.dataReady()) {
      int16_t d = sensor.distance();
      uint8_t st = 255;
      sensor.VL53L1X_GetRangeStatus(&st);
      uint16_t sg = 0;
      sensor.VL53L1X_GetSignalRate(&sg);
      sensor.clearInterrupt();
      if (verworfen < 1) { verworfen++; continue; }
      gelesen++;
      summeS += sg;
      if ((st == 0 || st == 1 || st == 2) && d > 0 && d < 2000) { summeD += d; gut++; }
    }
    delay(1);
  }
  sig = gelesen ? summeS / gelesen : 0;
  if (gut == 0) { dist = 0; return false; }
  dist = summeD / gut;
  return true;
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);
      sensor.setTimingBudget(ZONE_TIMING_MS);
      sensor.VL53L1X_SetInterMeasurementInMs(ZONE_TIMING_MS + 5);
      sensor.VL53L1X_SetSignalThreshold(SIGNAL_SCHWELLE);
      sensor.VL53L1X_SetSigmaThreshold(SIGMA_SCHWELLE);
      if (sensor.startRanging()) return true;
    }
    Serial.printf("Sensor-Start fehlgeschlagen (Versuch %d)\n", versuch);
    delay(300);
  }
  return false;
}

bool nullpunktSetzen() {
  zeige("0...");
  Serial.println("Nullpunkt: Holz/Tisch LEER lassen!");
  delay(500);
  int gut = 0;
  for (int z = 0; z < NZ; z++) {
    float d, s;
    baseOK[z] = messeZone(z, 8, d, s);
    baseD[z] = d;
    baseS[z] = s;
    zoneH[z] = 0;
    aktiv[z] = false;
    if (baseOK[z]) gut++;
    Serial.printf("Zone %2d (%2d,%2d): %6.1f mm | Signal %5.0f %s\n", z, POS[z % NPOS], POS[z / NPOS], d, s, baseOK[z] ? "" : "(ungueltig)");
  }
  if (gut < NZ / 2) return false;
  bloecke = 0; letzterKandidat = -1; stabil = 0; durchgang = 0;
  zeigeBloecke(0);
  return true;
}

void tasteAbfragen() {
  bool t = digitalRead(TASTE_PIN);
  if (tasteAlt == HIGH && t == LOW) {
    delay(30);
    if (digitalRead(TASTE_PIN) == LOW) {
      rohModus = !rohModus;
      if (rohModus) zeigeText("000c00");
      else zeigeBloecke(bloecke);
    }
  }
  tasteAlt = t;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!nullpunktSetzen()) zeige("ERR"); }
    else if (c == 'd') { debug = !debug; Serial.printf("Debug %s\n", debug ? "AN" : "AUS"); }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TASTE_PIN, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  display.begin();
  display.setIntensity(HELLIGKEIT);
  display.setTextAlignment(PA_CENTER);
  zeige("START");

  if (!sensorStarten()) {
    Serial.println("Sensor nicht gefunden! Verkabelung pruefen.");
    zeige("ERROR");
    while (true) delay(1000);
  }
  if (!nullpunktSetzen()) {
    Serial.println("Nullpunkt fehlgeschlagen - Sensor/Tisch pruefen.");
    zeige("ERR");
    while (true) delay(1000);
  }
  Serial.println("Bereit. k = Nullpunkt, d = Debug, BOOT-Taste = Messwerte");
}

void loop() {
  seriellBefehle();
  tasteAbfragen();

  bool irgendAktiv = false;
  for (int z = 0; z < NZ; z++) if (aktiv[z]) irgendAktiv = true;
  bool voll = !irgendAktiv || (durchgang % VOLLSCAN_ALLE == 0);
  durchgang++;

  for (int z = 0; z < NZ; z++) {
    if (!baseOK[z]) continue;
    if (!voll && !aktiv[z]) continue;
    float d, s;
    bool ok = messeZone(z, voll ? 2 : 4, d, s);
    float ratio = baseS[z] > 20 ? s / baseS[z] : 1.0f;
    float h = 0;
    if (ok) h = baseD[z] - d;
    if (h < 0) h = 0;
    bool dunkel = !ok || ratio < DUNKEL_SIGNAL;
    if (dunkel && h < EIN_MM) h = BLOCK_MM;
    zoneH[z] = h;
    zoneD[z] = ok ? d : 0;
    zoneRatio[z] = ratio;
    seriellBefehle();
  }

  float maxH = 0;
  int maxZ = 0;
  for (int z = 0; z < NZ; z++) {
    aktiv[z] = zoneH[z] >= AUS_MM;
    if (zoneH[z] > maxH) { maxH = zoneH[z]; maxZ = z; }
  }

  int kandidat;
  if (maxH < (bloecke > 0 ? AUS_MM : EIN_MM)) {
    kandidat = 0;
  } else {
    float nF = maxH / BLOCK_MM;
    kandidat = (int)lroundf(nF);
    if (kandidat < 1) kandidat = 1;
    if (kandidat > MAX_BLOECKE) kandidat = MAX_BLOECKE;
    if (bloecke > 0 && kandidat != bloecke && fabsf(nF - bloecke) < 0.5f + HYSTERESE) kandidat = bloecke;
  }

  if (kandidat == letzterKandidat) { if (stabil < 1000) stabil++; }
  else { letzterKandidat = kandidat; stabil = 1; }

  if (kandidat != bloecke && stabil >= STABIL_NOETIG) {
    bloecke = kandidat;
    zeigeBloecke(bloecke);
  }

  if (rohModus) {
    int dm = (int)lroundf(zoneD[maxZ]);
    int sp = (int)lroundf(100.0f * zoneRatio[maxZ]);
    if (dm > 999) dm = 999;
    if (sp > 99) sp = 99;
    char t[12];
    snprintf(t, sizeof(t), "%03dc%02d", dm, sp);
    zeigeText(t);
  }

  if (debug) {
    Serial.printf("%s | max Zone %2d: Hoehe %5.1f mm | Signal %3.0f%% | -> %d Bloecke\n",
                  voll ? "Vollscan" : "Tracking", maxZ, maxH, 100.0f * zoneRatio[maxZ], bloecke);
  }
}
