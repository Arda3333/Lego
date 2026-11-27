#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
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
#define TIMING_MS        33
#define SIGNAL_SCHWELLE  300
#define SIGMA_SCHWELLE   30

#define NPOS             4
#define NZ               16
const int POS[NPOS] = {3, 6, 9, 12};

#define CAL_SAMPLES      16
#define SCAN_SAMPLES     2
#define TRACK_SAMPLES    6
#define TRACK_ZONEN      2
#define VOLLSCAN_ALLE    4
#define CLUSTER_BREITE   10
#define EIN_MM           6.0f
#define NULL_BAND_MM     4.0f
#define ANZEIGE_HYST_MM  1.5f

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
Preferences speicher;

float baseD[NZ];
bool  baseOK[NZ];
float zoneH[NZ];
float zoneD[NZ];

float faktor = 1.0f;
float letzteRohHoehe = 0;
float angezeigtMM = -1;
int   nullZaehler = 0;
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

void zeigeHoehe(float mm) {
  int zehntel = (int)lroundf(mm);
  if (zehntel < 0) zehntel = 0;
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", zehntel / 10, zehntel % 10);
  if (!rohModus) zeigeText(text);
}

uint8_t spadNummer(int col, int row) {
  if (row < 8) return 128 + 8 * col + row;
  return 127 - 8 * col - (row - 8);
}

void sortiere(int *w, int n) {
  for (int i = 1; i < n; i++) {
    int x = w[i], j = i - 1;
    while (j >= 0 && w[j] > x) { w[j + 1] = w[j]; j--; }
    w[j + 1] = x;
  }
}

bool messeZone(int z, int nSamples, float &dist) {
  sensor.stopRanging();
  sensor.VL53L1X_SetROI(ROI_GROESSE, ROI_GROESSE);
  sensor.VL53L1X_SetROICenter(spadNummer(POS[z % NPOS], POS[z / NPOS]));
  sensor.startRanging();
  int w[CAL_SAMPLES];
  int gut = 0, verworfen = 0, gelesen = 0;
  unsigned long start = millis();
  while (gelesen < nSamples && millis() - start < 600) {
    if (sensor.dataReady()) {
      int16_t d = sensor.distance();
      uint8_t st = 255;
      sensor.VL53L1X_GetRangeStatus(&st);
      sensor.clearInterrupt();
      if (verworfen < 1) { verworfen++; continue; }
      gelesen++;
      if ((st == 0 || st == 1 || st == 2) && d > 0 && d < 2000) w[gut++] = d;
    }
    delay(1);
  }
  if (gut == 0) return false;
  sortiere(w, gut);
  int noetig = (nSamples >= TRACK_SAMPLES) ? 2 : 1;
  if (gut < noetig) return false;
  for (int i = 0; i + noetig <= gut; i++) {
    long summe = 0;
    int j = i;
    while (j < gut && w[j] - w[i] <= CLUSTER_BREITE) { summe += w[j]; j++; }
    if (j - i >= noetig) { dist = (float)summe / (j - i); return true; }
  }
  return false;
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);
      sensor.setTimingBudget(TIMING_MS);
      sensor.VL53L1X_SetInterMeasurementInMs(TIMING_MS + 5);
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
    float d = 0;
    baseOK[z] = messeZone(z, CAL_SAMPLES, d);
    baseD[z] = d;
    zoneH[z] = 0;
    zoneD[z] = d;
    if (baseOK[z]) gut++;
    Serial.printf("Zone %2d (%2d,%2d): %6.1f mm %s\n", z, POS[z % NPOS], POS[z / NPOS], d, baseOK[z] ? "" : "(ungueltig)");
  }
  if (gut < NZ / 2) return false;
  angezeigtMM = 0;
  nullZaehler = 0;
  durchgang = 0;
  zeigeHoehe(0);
  return true;
}

void tasteAbfragen() {
  bool t = digitalRead(TASTE_PIN);
  if (tasteAlt == HIGH && t == LOW) {
    delay(30);
    if (digitalRead(TASTE_PIN) == LOW) {
      rohModus = !rohModus;
      if (rohModus) zeigeText("000c00");
      else zeigeHoehe(angezeigtMM);
    }
  }
  tasteAlt = t;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!nullpunktSetzen()) zeige("ERR"); }
    else if (c == 'd') { debug = !debug; Serial.printf("Debug %s\n", debug ? "AN" : "AUS"); }
    else if (c == 'h') {
      float soll = Serial.parseFloat();
      if (soll >= 5 && soll <= 600 && letzteRohHoehe > 5) {
        faktor = soll / letzteRohHoehe;
        speicher.putFloat("fak", faktor);
        Serial.printf("Eichung: soll %.1f mm, gemessen %.1f mm -> Faktor %.4f\n", soll, letzteRohHoehe, faktor);
      } else Serial.println("Eichung: Gegenstand bekannter Hoehe auflegen, dann z.B. h37 (mm)");
    }
    else if (c == 'f') { faktor = 1.0f; speicher.putFloat("fak", faktor); Serial.println("Faktor zurueckgesetzt"); }
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

  speicher.begin("zonen", false);
  faktor = speicher.getFloat("fak", 1.0f);

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
  Serial.println("Bereit. k = Nullpunkt, d = Debug, h37 = Eichung mit 37 mm hohem Gegenstand, f = Eichung loeschen");
}

void loop() {
  seriellBefehle();
  tasteAbfragen();

  bool irgendAktiv = false;
  for (int z = 0; z < NZ; z++) if (zoneH[z] >= EIN_MM) irgendAktiv = true;
  bool voll = !irgendAktiv || (durchgang % VOLLSCAN_ALLE == 0);
  durchgang++;

  if (voll) {
    for (int z = 0; z < NZ; z++) {
      if (!baseOK[z]) continue;
      float d;
      bool ok = messeZone(z, SCAN_SAMPLES, d);
      float h = ok ? baseD[z] - d : 0;
      if (h < 0) h = 0;
      zoneH[z] = h;
      zoneD[z] = ok ? d : 0;
      seriellBefehle();
    }
  }

  int fokus[TRACK_ZONEN];
  int nFokus = 0;
  bool benutzt[NZ];
  for (int z = 0; z < NZ; z++) benutzt[z] = false;
  for (int k = 0; k < TRACK_ZONEN; k++) {
    int best = -1;
    for (int z = 0; z < NZ; z++)
      if (!benutzt[z] && zoneH[z] >= EIN_MM && (best < 0 || zoneH[z] > zoneH[best])) best = z;
    if (best < 0) break;
    benutzt[best] = true;
    fokus[nFokus++] = best;
  }
  for (int k = 0; k < nFokus; k++) {
    int z = fokus[k];
    float d;
    bool ok = messeZone(z, TRACK_SAMPLES, d);
    float h = ok ? baseD[z] - d : 0;
    if (h < 0) h = 0;
    zoneH[z] = h;
    zoneD[z] = ok ? d : 0;
  }

  float maxH = 0;
  int maxZ = 0;
  for (int z = 0; z < NZ; z++) {
    if (zoneH[z] > maxH) { maxH = zoneH[z]; maxZ = z; }
  }
  letzteRohHoehe = maxH;
  float kandidat = maxH * faktor;
  if (kandidat < NULL_BAND_MM) kandidat = 0;

  bool nullWechsel = (kandidat == 0) != (angezeigtMM == 0);
  if (nullWechsel) {
    if (++nullZaehler >= 2) { angezeigtMM = kandidat; nullZaehler = 0; zeigeHoehe(angezeigtMM); }
  } else {
    nullZaehler = 0;
    if (fabsf(kandidat - angezeigtMM) >= ANZEIGE_HYST_MM) { angezeigtMM = kandidat; zeigeHoehe(angezeigtMM); }
  }

  if (rohModus) {
    int dm = (int)lroundf(zoneD[maxZ]);
    if (dm > 999) dm = 999;
    char t[12];
    snprintf(t, sizeof(t), "%03dc%02d", dm, maxZ);
    zeigeText(t);
  }

  if (debug) {
    Serial.printf("%s | Zone %2d: Abstand %6.1f mm | Hoehe roh %5.1f mm | Anzeige %5.1f mm\n",
                  voll ? "Vollscan" : "Tracking", maxZ, zoneD[maxZ], maxH, angezeigtMM);
  }
}
