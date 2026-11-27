// Turmhoehe per Dauer-Scan: ESP32 + VL53L1X (von oben) + 5x MAX7219 (FC16)
// - Der Sensor scannt dauernd 16 Messfeld-Positionen (4x4) ueber die Ablageflaeche.
// - Beim Start wird pro Position der Abstand zum LEEREN Tisch gemessen (Nullpunkt).
// - Wird ein Block aufgelegt, wird der Abstand dort kuerzer: Hoehe = Nullpunkt - Abstand.
// - Angezeigt wird die gemessene Hoehe (kein Zaehlen). Kein Block = 0.0cm.
// Serial (115200): k = Nullpunkt neu (Tisch leer!) | d = Debug an/aus

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_VL53L1X.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

// ---------------- LED MATRIX ----------------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   4
#define DATA_PIN      23
#define CS_PIN        5
#define CLK_PIN       18
#define HELLIGKEIT    4

// ---------------- SENSOR --------------------
#define SDA_PIN          21
#define SCL_PIN          22
#define TIMING_MS        33     // Messzeit pro Einzelmessung
#define ROI_GROESSE      4      // engstes Messfeld
#define SIGNAL_SCHWELLE  300    // kcps (niedrig -> auch dunkle/blaue Bloecke)
#define SIGMA_SCHWELLE   30     // mm

// ---------------- SCAN ----------------------
#define NPOS             4
const int POS[NPOS] = {3, 6, 9, 12};      // Pixel-Positionen (0..15) in Spalte und Zeile
#define NZONEN           (NPOS * NPOS)
#define MESS_N           6      // Messungen pro Position im Scan
#define CAL_N            16     // Messungen pro Position fuer den Nullpunkt
#define CLUSTER_BREITE   10.0f  // mm: Werte innerhalb gelten als dieselbe Flaeche
#define MIN_TREFFER      2      // so viele Messungen muessen die naechste Flaeche bestaetigen
#define VOLLSCAN_ALLE    5      // bei Block: nur aktive Positionen, aber jeder 5. Durchgang alle

// ---------------- ANZEIGE -------------------
#define ZERO_BAND_MM     4.0f   // darunter = kein Block -> 0.0
#define ANZEIGE_HYST_MM  1.5f   // Anzeige aendert sich erst ab dieser Differenz

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

float baseMM[NZONEN];
bool  baseOK[NZONEN];
float zoneH[NZONEN];            // aktuelle Hoehe pro Position (mm)
bool  aktiv[NZONEN];
float angezeigtMM = -1;
int   wechsel = 0;
int   durchgang = 0;
bool  debug = false;
uint8_t letzterStatus = 0;

// ============================================================
void zeige(const char *text) {              // mittig (START, ERR, ...)
  display.setTextAlignment(PA_CENTER);
  display.displayClear();
  display.print(text);
}

// Hoehe: mittig zeichnen, dann um VERSCHIEBUNG_SPALTEN LED-Spalten schieben.
//   positiv = nach rechts, negativ = nach links, 0 = mittig.
//   Falls es in die falsche Richtung geht: Vorzeichen umdrehen (3 -> -3).
#define VERSCHIEBUNG_SPALTEN 3
void zeigeHoehe(float mm) {
  int zehntel = (int)lroundf(mm);          // mm = Zehntel cm
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", zehntel / 10, zehntel % 10);
  display.setTextAlignment(PA_CENTER);
  display.displayClear();
  display.print(text);
  MD_MAX72XX *mx = display.getGraphicObject();
  for (int i = 0; i < abs(VERSCHIEBUNG_SPALTEN); i++)
    mx->transform(VERSCHIEBUNG_SPALTEN > 0 ? MD_MAX72XX::TSR : MD_MAX72XX::TSL);
}

void sortiere(float *w, int n) {
  for (int i = 1; i < n; i++) {
    float x = w[i];
    int j = i - 1;
    while (j >= 0 && w[j] > x) { w[j + 1] = w[j]; j--; }
    w[j + 1] = x;
  }
}

uint8_t spadNummer(int col, int row) {     // Pixel -> SPAD-Nummer (Mitte = 199)
  if (row < 8) return 128 + 8 * col + row;
  return 127 - 8 * col - (row - 8);
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

// Nur die NAECHSTE Flaeche zaehlt (Block ist naeher als der Tisch).
bool naechsteFlaeche(const float *werte, int n, int noetig, float &ergebnis) {
  float s[40];
  if (n > 40) n = 40;
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

// Messfeld auf Position z richten und Abstand messen.
bool messeZone(int z, int n, int noetig, float &out) {
  sensor.stopRanging();
  sensor.VL53L1X_SetROI(ROI_GROESSE, ROI_GROESSE);                 // erst Groesse ...
  sensor.VL53L1X_SetROICenter(spadNummer(POS[z % NPOS], POS[z / NPOS]));   // ... dann Mitte
  sensor.startRanging();
  float w[24];
  if (n > 24) n = 24;
  int ok = 0, verworfen = 0;
  unsigned long start = millis();
  while (ok < n && millis() - start < 1500) {
    float d;
    if (leseAbstand(d)) { if (verworfen < 1) verworfen++; else w[ok++] = d; }
    delay(1);
  }
  if (ok < noetig) return false;
  return naechsteFlaeche(w, ok, noetig, out);
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);                       // Short
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

// ============================================================
//  Nullpunkt: Abstand zum leeren Tisch an jeder Position
// ============================================================
bool kalibrieren() {
  zeige("0...");
  Serial.println("Nullpunkt: Tisch LEER lassen!");
  delay(500);
  int gut = 0;
  for (int z = 0; z < NZONEN; z++) {
    float d;
    baseOK[z] = messeZone(z, CAL_N, 6, d);
    baseMM[z] = baseOK[z] ? d : 0;
    zoneH[z] = 0;
    aktiv[z] = false;
    if (baseOK[z]) gut++;
    Serial.printf("Pos %2d (%2d,%2d): %6.1f mm %s\n", z, POS[z % NPOS], POS[z / NPOS], baseMM[z], baseOK[z] ? "" : "(ungueltig)");
  }
  if (gut < NZONEN / 2) return false;
  angezeigtMM = 0;
  wechsel = 0;
  durchgang = 0;
  zeigeHoehe(0);
  Serial.printf("Nullpunkt fertig (%d/%d Positionen). Bereit.\n", gut, NZONEN);
  return true;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!kalibrieren()) zeige("ERR"); }
    else if (c == 'd') { debug = !debug; Serial.printf("Debug %s\n", debug ? "AN" : "AUS"); }
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
    Serial.println("Nullpunkt fehlgeschlagen!");
    zeige("ERR");
    while (true) delay(1000);
  }
  Serial.println("k = Nullpunkt neu, d = Debug");
}

void loop() {
  seriellBefehle();

  // Dauer-Scan: ohne Block alle Positionen; mit Block nur die aktiven (schnell, genau),
  // jeder 5. Durchgang trotzdem alle (damit neue Bloecke gefunden werden).
  bool irgendAktiv = false;
  for (int z = 0; z < NZONEN; z++) if (aktiv[z]) irgendAktiv = true;
  bool voll = !irgendAktiv || (durchgang % VOLLSCAN_ALLE == 0);
  durchgang++;

  for (int z = 0; z < NZONEN; z++) {
    if (!baseOK[z]) continue;
    if (!voll && !aktiv[z]) continue;
    float d;
    if (messeZone(z, MESS_N, MIN_TREFFER, d)) zoneH[z] = baseMM[z] - d;   // neuer Abstand -> Hoehe
    seriellBefehle();
  }

  float maxH = 0;
  int maxZ = -1;
  for (int z = 0; z < NZONEN; z++) {
    aktiv[z] = zoneH[z] > ZERO_BAND_MM;
    if (zoneH[z] > maxH) { maxH = zoneH[z]; maxZ = z; }
  }
  float kandidat = (maxH > ZERO_BAND_MM) ? maxH : 0;

  // 0 <-> Block erst nach 2 Durchgaengen in Folge (gegen Ausreisser), sonst mit Hysterese
  bool nullWechsel = (kandidat == 0) != (angezeigtMM == 0);
  if (nullWechsel) {
    if (++wechsel >= 2) { angezeigtMM = kandidat; zeigeHoehe(angezeigtMM); wechsel = 0; }
  } else {
    wechsel = 0;
    if (fabsf(kandidat - angezeigtMM) >= ANZEIGE_HYST_MM) { angezeigtMM = kandidat; zeigeHoehe(angezeigtMM); }
  }

  if (debug || maxZ >= 0) {
    Serial.printf("Hoehe: %5.1f mm (Pos %d) | Anzeige: %5.1f mm | %s\n",
                  maxH, maxZ, angezeigtMM, voll ? "Vollscan" : "Block-Tracking");
  }
}
