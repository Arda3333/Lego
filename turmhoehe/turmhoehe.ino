// Turmhoehe: 0 Bloecke = 0.0cm, jeder Block +3.7cm (weisse UND blaue Bloecke)
// Sensor: VL53L1X von oben, Anzeige: 5x MAX7219 (FC16)
//
// Serial Monitor (115200): k = Nullpunkt neu | b37 = Blockhoehe in mm | d = Debug an/aus
//   p8,7  = Messfeld auf Pixel (Spalte 0-15, Zeile 0-15) des Sensors richten (8,7 = Mitte)
//   r4    = Groesse des Messfelds (4..16)
// Danach wird der Nullpunkt automatisch neu gemessen (Tisch leer!).

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
#define SDA_PIN       21
#define SCL_PIN       22
#define TIMING_BUDGET_MS  100    // lange Messzeit = mehr Signal (wichtig fuer blaue Bloecke)
#define ROI_GROESSE   4          // 4 = engstes Blickfeld (Block fuellt mehr davon); bei Fehlern mit blauen Bloecken 6 probieren
#define SIGNAL_SCHWELLE  300     // kcps, niedrig -> dunkle/blaue Flaechen werden akzeptiert
#define SIGMA_SCHWELLE   30      // mm

// ---------------- BLOECKE -------------------
#define MAX_BLOECKE   17
float blockMM = 37.0;            // Hoehe eines Blocks in mm (zur Laufzeit: b37)

// ---------------- MESSUNG -------------------
#define FENSTER        25        // letzte Messwerte (~2,5 s)
#define CLUSTER_BREITE 10.0f     // mm: Werte innerhalb gelten als "dieselbe Flaeche"
#define CLUSTER_MIN    2         // so wenige Treffer reichen: ein Block fuellt nur einen kleinen Teil des Messflecks
#define STABIL_AUF     4         // gleiche Ergebnisse hintereinander, bevor ein Block HINZUKOMMT (schnell)
#define STABIL_AB      6         // ... bevor ein Block WEGFAELLT (traeger: ein Block geht nicht kurz verloren)
#define HYSTERESE      0.15f     // Block-Anteil Reserve gegen Hin-und-Her-Springen

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

int   roiCol = 8, roiRow = 7, roiGroesse = ROI_GROESSE;
float baselineMM = 0;            // Abstand Sensor -> leerer Tisch
float fenster[FENSTER];
int   fN = 0, fIdx = 0;
int   angezeigt = -1;
int   letzterKandidat = -1;
int   stabil = 0;
bool  debug = false;
uint8_t letzterStatus = 0;

// ============================================================
//  Anzeige
// ============================================================
void zeige(const char *text) {
  display.displayClear();
  display.print(text);
}

void zeigeBloecke(int bloecke) {
  int zehntel = (int)lroundf(bloecke * blockMM);   // mm = Zehntel cm
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", zehntel / 10, zehntel % 10);
  zeige(text);
  Serial.printf(">> %d Bloecke = %s\n", bloecke, text);
}

// ============================================================
//  Sensor
// ============================================================
void sortiere(float *w, int n) {
  for (int i = 1; i < n; i++) {
    float x = w[i];
    int j = i - 1;
    while (j >= 0 && w[j] > x) { w[j + 1] = w[j]; j--; }
    w[j + 1] = x;
  }
}

bool nullpunktSetzen();

bool leseAbstand(float &mm) {
  if (!sensor.dataReady()) return false;
  int16_t d = sensor.distance();
  uint8_t status = 255;
  sensor.VL53L1X_GetRangeStatus(&status);
  sensor.clearInterrupt();
  letzterStatus = status;
  // 0 = gut, 1 = Sigma-Warnung, 2 = schwaches Signal (typisch bei blau)
  bool ok = (status == 0 || status == 1 || status == 2);
  if (!ok || d <= 0 || d > 2000) return false;
  mm = d;
  return true;
}

// Der Sensor sieht mit seinem Blickfeld oft Block UND Tisch gleichzeitig; die Messwerte
// springen dann zwischen beiden. Darum: sortieren und die NAECHSTE Gruppe nehmen
// (kleinster Abstand), die von genug Messungen bestaetigt wird.
bool naechsteFlaeche(const float *werte, int n, int noetig, float &ergebnis) {
  float s[60];
  if (n > 60) n = 60;
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

// Pixel (Spalte/Zeile 0..15) -> SPAD-Nummer des VL53L1X (Mitte = 199)
uint8_t spadNummer(int col, int row) {
  if (row < 8) return 128 + 8 * col + row;
  return 127 - 8 * col - (row - 8);
}

void roiAnwenden() {
  sensor.stopRanging();
  sensor.VL53L1X_SetROI(roiGroesse, roiGroesse);
  sensor.VL53L1X_SetROICenter(spadNummer(roiCol, roiRow));
  sensor.startRanging();
  Serial.printf("Messfeld: Pixel %d,%d (SPAD %d), Groesse %d\n", roiCol, roiRow, spadNummer(roiCol, roiRow), roiGroesse);
  if (!nullpunktSetzen()) zeige("ERR");
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);              // Short
      sensor.VL53L1X_SetROI(ROI_GROESSE, ROI_GROESSE);
      sensor.setTimingBudget(TIMING_BUDGET_MS);
      sensor.VL53L1X_SetInterMeasurementInMs(TIMING_BUDGET_MS + 5);
      sensor.VL53L1X_SetSignalThreshold(SIGNAL_SCHWELLE);
      sensor.VL53L1X_SetSigmaThreshold(SIGMA_SCHWELLE);
      if (sensor.startRanging()) return true;
    }
    Serial.printf("Sensor-Start fehlgeschlagen (Versuch %d)\n", versuch);
    delay(300);
  }
  return false;
}

// ============================================================
//  Nullpunkt
// ============================================================
bool nullpunktSetzen() {
  zeige("0...");
  Serial.println("Nullpunkt: nichts unter den Sensor stellen!");
  delay(500);

  const int N = 50;
  float w[N];
  int ok = 0;
  unsigned long start = millis();
  while (ok < N && millis() - start < 15000) {
    float d;
    if (leseAbstand(d)) w[ok++] = d;
    delay(1);
  }
  float ergebnis;
  if (ok < 20 || !naechsteFlaeche(w, ok, (int)(ok * 0.4f), ergebnis)) return false;

  baselineMM = ergebnis;
  fN = 0; fIdx = 0;
  stabil = 0; letzterKandidat = -1;
  angezeigt = 0;
  Serial.printf("Boden-Abstand: %.1f mm\n", baselineMM);
  zeigeBloecke(0);
  return true;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!nullpunktSetzen()) zeige("ERR"); }
    else if (c == 'b') {
      int v = Serial.parseInt();
      if (v >= 5 && v <= 200) {
        blockMM = v;
        Serial.printf("Blockhoehe: %d mm\n", v);
        angezeigt = -1; stabil = 0; letzterKandidat = -1;
      } else Serial.println("Ungueltige Blockhoehe (5..200 mm), z.B. b37");
    }
    else if (c == 'p') {
      int col = Serial.parseInt();
      int row = Serial.parseInt();
      if (col >= 0 && col <= 15 && row >= 0 && row <= 15) { roiCol = col; roiRow = row; roiAnwenden(); }
      else Serial.println("Ungueltig, z.B. p8,7");
    }
    else if (c == 'r') {
      int g = Serial.parseInt();
      if (g >= 4 && g <= 16) { roiGroesse = g; roiAnwenden(); }
      else Serial.println("Ungueltig, z.B. r4 (4..16)");
    }
    else if (c == 'd') {
      debug = !debug;
      Serial.printf("Debug %s\n", debug ? "AN" : "AUS");
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

  if (!nullpunktSetzen()) {
    Serial.println("Keine gueltige Messung beim Start!");
    zeige("ERR");
    while (true) delay(1000);
  }
  Serial.println("Bereit. Bloecke auflegen. (k = Nullpunkt, b37 = Blockhoehe, d = Debug)");
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
  if (!naechsteFlaeche(fenster, FENSTER, CLUSTER_MIN, flaeche)) {
    if (debug) Serial.println("keine stabile Flaeche");
    return;
  }

  float h = baselineMM - flaeche;          // Hoehe ueber dem Tisch (mm)
  if (h < 0) h = 0;

  // Nullpunkt langsam nachfuehren (Temperatur), nur wenn leer
  if (angezeigt == 0 && h < 8) baselineMM -= 0.002f * h;   // Richtung aktueller Messwert
  float nF = h / blockMM;                  // Bloecke als Kommazahl
  int kandidat = (int)lroundf(nF);
  if (kandidat > MAX_BLOECKE) kandidat = MAX_BLOECKE;

  // Hysterese: nur wechseln, wenn klar naeher an der neuen Stufe
  if (angezeigt >= 0 && kandidat != angezeigt && fabsf(nF - angezeigt) < 0.5f + HYSTERESE)
    kandidat = angezeigt;

  if (kandidat == letzterKandidat) { if (stabil < 1000) stabil++; }
  else { letzterKandidat = kandidat; stabil = 1; }

  if (debug) {
    Serial.printf("Flaeche: %6.1f mm  Hoehe: %5.1f mm  = %.2f Bloecke  Status: %d  stabil: %d -> %d\n",
                  flaeche, h, nF, letzterStatus, stabil, kandidat);
  }

  int noetig = (kandidat > angezeigt) ? STABIL_AUF : STABIL_AB;
  if (stabil >= noetig && kandidat != angezeigt) {
    angezeigt = kandidat;
    zeigeBloecke(angezeigt);
  }
}
