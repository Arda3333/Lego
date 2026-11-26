// Turmhoehe EINFACH: ESP32 + VL53L1X (von oben) + 5x MAX7219 (FC16)
// Misst dauernd den Abstand. Kein Block = 0.0cm. Block drauf -> Abstand wird kuerzer
// -> Hoehe = Nullpunkt - Abstand (in cm). Kein Zaehlen.
//
// Serial (115200): k = Nullpunkt neu (Tisch leer!) | r6 = Messfeld-Groesse 4..16 | d = Debug an/aus

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_VL53L1X.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

// ---------------- LED MATRIX ----------------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   5
#define DATA_PIN      23
#define CS_PIN        5
#define CLK_PIN       18
#define HELLIGKEIT    4

// ---------------- SENSOR --------------------
#define SDA_PIN          21
#define SCL_PIN          22
#define TIMING_MS        100    // lange Messzeit = mehr Signal (auch blaue Bloecke)
#define SIGNAL_SCHWELLE  300
#define SIGMA_SCHWELLE   30
int roiGroesse = 6;             // 4 = engstes Messfeld ... 16 = breitestes (zur Laufzeit: r6)

// ---------------- MESSUNG -------------------
#define FENSTER          16     // letzte Messwerte (~1,6 s)
#define CLUSTER_BREITE   10.0f  // mm: Werte innerhalb gelten als dieselbe Flaeche
#define MIN_TREFFER      2      // so viele Messungen muessen die naechste Flaeche bestaetigen
#define ZERO_BAND_MM     4.0f   // darunter = kein Block -> 0.0
#define ANZEIGE_HYST_MM  1.5f   // Anzeige aendert sich erst ab dieser Differenz

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

float fenster[FENSTER];
int   fN = 0, fIdx = 0;
float baseMM = 0;               // Abstand Sensor -> leerer Tisch
float angezeigtMM = 0;
int   wechsel = 0;
int   zaehler = 0;
bool  debug = true;
uint8_t letzterStatus = 0;

// ============================================================
void zeige(const char *text) {              // mittig (START, ERR, ...)
  display.setTextAlignment(PA_CENTER);
  display.displayClear();
  display.print(text);
}

// Hoehe: linksbuendig, mit Leerzeichen davor = nach rechts geschoben.
// 1 Leerzeichen = 3 LED-Spalten. Mehr/weniger verschieben: Zahl aendern (0 = ganz links).
#define VERSCHIEBUNG_LEERZEICHEN 1
void zeigeLinks(const char *text) {
  display.setTextAlignment(PA_LEFT);
  display.displayClear();
  display.print(text);
}

void zeigeHoehe(float mm) {
  int zehntel = (int)lroundf(mm);          // mm = Zehntel cm
  char text[16];
  snprintf(text, sizeof(text), "%*s%d.%dcm", VERSCHIEBUNG_LEERZEICHEN, "", zehntel / 10, zehntel % 10);
  zeigeLinks(text);
}

void sortiere(float *w, int n) {
  for (int i = 1; i < n; i++) {
    float x = w[i];
    int j = i - 1;
    while (j >= 0 && w[j] > x) { w[j + 1] = w[j]; j--; }
    w[j + 1] = x;
  }
}

bool leseAbstand(float &mm) {
  if (!sensor.dataReady()) return false;
  int16_t d = sensor.distance();
  uint8_t status = 255;
  sensor.VL53L1X_GetRangeStatus(&status);
  sensor.clearInterrupt();
  letzterStatus = status;
  bool ok = (status == 0 || status == 1 || status == 2);   // 2 = schwaches Signal (blau)
  if (!ok || d <= 0 || d > 2000) return false;
  mm = d;
  return true;
}

// Nur die NAECHSTE Flaeche zaehlt (ein Block ist naeher als der Tisch), auch wenn
// der Sensor oft noch den Tisch mitsieht.
bool naechsteFlaeche(const float *werte, int n, int noetig, float &ergebnis) {
  float s[64];
  if (n > 64) n = 64;
  for (int i = 0; i < n; i++) s[i] = werte[i];
  sortiere(s, n);
  for (int i = 0; i + noetig <= n; i++) {
    float summe = 0;
    int j = i;
    while (j < n && s[j] - s[i] <= CLUSTER_BREITE) { summe += s[j]; j++; }
    if (j - i >= noetig) { ergebnis = summe / (j - i); return true; }
  }
  return false;
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);                        // Short
      sensor.VL53L1X_SetROI(roiGroesse, roiGroesse);
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

// Nullpunkt: Abstand zum LEEREN Tisch
bool kalibrieren() {
  zeige("0...");
  Serial.println("Nullpunkt: Tisch LEER lassen!");
  delay(500);
  const int N = 40;
  float w[N];
  int ok = 0;
  unsigned long start = millis();
  while (ok < N && millis() - start < 15000) {
    float d;
    if (leseAbstand(d)) w[ok++] = d;
    delay(1);
  }
  float e;
  if (ok < 20 || !naechsteFlaeche(w, ok, (int)(ok * 0.4f), e)) return false;
  baseMM = e;
  fN = 0; fIdx = 0;
  angezeigtMM = 0; wechsel = 0;
  zeigeHoehe(0);
  Serial.printf("Nullpunkt: %.1f mm  (Messfeld %d)\n", baseMM, roiGroesse);
  return true;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!kalibrieren()) zeige("ERR"); }
    else if (c == 'd') { debug = !debug; Serial.printf("Debug %s\n", debug ? "AN" : "AUS"); }
    else if (c == 'r') {
      int g = Serial.parseInt();
      if (g >= 4 && g <= 16) {
        roiGroesse = g;
        sensor.stopRanging();
        sensor.VL53L1X_SetROI(roiGroesse, roiGroesse);
        sensor.startRanging();
        if (!kalibrieren()) zeige("ERR");
      } else Serial.println("Ungueltig, z.B. r6 (4..16)");
    }
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
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
  if (!kalibrieren()) {
    Serial.println("Nullpunkt fehlgeschlagen - Sensor/Tisch pruefen.");
    zeige("ERR");
    while (true) delay(1000);
  }
  Serial.println("Bereit. k = Nullpunkt, r6 = Messfeld, d = Debug");
}

void loop() {
  seriellBefehle();

  float d;
  if (!leseAbstand(d)) return;

  fenster[fIdx] = d;
  fIdx = (fIdx + 1) % FENSTER;
  if (fN < FENSTER) fN++;
  if (fN < FENSTER) return;

  float flaeche;
  if (!naechsteFlaeche(fenster, FENSTER, MIN_TREFFER, flaeche)) return;

  float h = baseMM - flaeche;                         // Hoehe ueber dem Tisch (mm)
  float kandidat = (h > ZERO_BAND_MM) ? h : 0;

  // 0 <-> Block erst nach 3 gleichen Ergebnissen in Folge, sonst mit Hysterese
  bool nullWechsel = (kandidat == 0) != (angezeigtMM == 0);
  if (nullWechsel) {
    if (++wechsel >= 3) { angezeigtMM = kandidat; zeigeHoehe(angezeigtMM); wechsel = 0; }
  } else {
    wechsel = 0;
    if (fabsf(kandidat - angezeigtMM) >= ANZEIGE_HYST_MM) { angezeigtMM = kandidat; zeigeHoehe(angezeigtMM); }
  }

  if (debug && (++zaehler % 5 == 0)) {
    Serial.printf("roh: %6.1f | Flaeche: %6.1f | Hoehe: %5.1f mm | Anzeige: %5.1f mm | Status: %d\n",
                  d, flaeche, h, angezeigtMM, letzterStatus);
  }
}
