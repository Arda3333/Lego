// Turmhoehe FINAL: weisse, blaue und DUNKELBLAUE Bloecke. 0 Bloecke = 0.0cm, jeder Block +3.7cm.
// Der Sensor wertet 3 Hinweise aus (Fenster der letzten 24 Messungen):
//   A) Abstand: die NAECHSTE Flaeche (Block ist naeher als der Tisch)
//   B) Signalstaerke: ein dunkler Block "schluckt" das Infrarot -> Signal sinkt stark
//   C) Anteil gueltiger Messungen: bei dunklen Bloecken fallen viele Messungen aus
// Wird A nicht gefunden, aber B oder C schlagen an, gilt das als (dunkler) Block.
// Serial (115200): k = Nullpunkt neu (Tisch leer!) | d = Debug an/aus

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
#define TIMING_MS        100     // lange Messzeit = mehr Signal
#define ROI_GROESSE      6
#define SIGNAL_SCHWELLE  300
#define SIGMA_SCHWELLE   30

// ---------------- BLOECKE -------------------
#define BLOCK_MM         37      // 3,7 cm pro Block
#define MAX_BLOECKE      17

// ---------------- MESSUNG -------------------
#define N_FENSTER        24      // letzte Messungen (~2,4 s)
#define CLUSTER_BREITE   10     // mm
#define MIN_TREFFER      2
#define STUFE_MIN_MM     8.0f    // so viel naeher = Block
#define DUNKEL_SIGNAL    0.6f    // Signal unter 60 % vom Leer-Wert -> dunkler Block
#define DUNKEL_GUELTIG   0.35f   // gueltige Messungen um so viel weniger als leer -> dunkler Block
#define STABIL_TOL_MM    5.0f
#define STABIL_NOETIG    8       // ruhige Auswertungen in Folge (~0,8 s)

// ============================================================
//  Schmale Zahlen-Schrift (nur . 0-9 c m)
// ============================================================
#define LEER 0,
MD_MAX72XX::fontType_t zahlenFont[] PROGMEM = {
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   //  0- 9
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 10-19
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 20-29
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 30-39
  LEER LEER LEER LEER LEER LEER                       // 40-45
  1, 0x40,                         // 46 '.'
  LEER                             // 47
  4, 0x3E, 0x41, 0x41, 0x3E,       // 48 '0'
  3, 0x42, 0x7F, 0x40,             // 49 '1'
  4, 0x62, 0x51, 0x49, 0x46,       // 50 '2'
  4, 0x22, 0x41, 0x49, 0x36,       // 51 '3'
  4, 0x18, 0x14, 0x12, 0x7F,       // 52 '4'
  4, 0x27, 0x45, 0x45, 0x39,       // 53 '5'
  4, 0x3E, 0x49, 0x49, 0x32,       // 54 '6'
  4, 0x01, 0x71, 0x09, 0x07,       // 55 '7'
  4, 0x36, 0x49, 0x49, 0x36,       // 56 '8'
  4, 0x26, 0x49, 0x49, 0x3E,       // 57 '9'
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 58-67
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 68-77
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 78-87
  LEER LEER LEER LEER LEER LEER LEER LEER LEER LEER   // 88-97
  LEER                                                // 98
  3, 0x38, 0x44, 0x44,             // 99 'c'
  LEER LEER LEER LEER LEER LEER LEER LEER LEER        // 100-108
  5, 0x7C, 0x04, 0x78, 0x04, 0x78, // 109 'm'
};

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Fenster der letzten Messungen
int      fDist[N_FENSTER];
bool     fGut[N_FENSTER];
uint16_t fSig[N_FENSTER];
int fIdx = 0, fN = 0;

// Leer-Werte (Nullpunkt)
float baseDist = 0;      // Abstand zum leeren Tisch (mm)
float baseGut  = 1;      // Anteil gueltiger Messungen bei leerem Tisch
float baseSig  = 0;      // mittlere Signalstaerke bei leerem Tisch

// Zaehl-Zustand
int   bloecke = 0;
float level = 0;         // "effektiver Abstand" beim aktuellen Stand
float anker = 0;
int   stabil = 0;
uint8_t letzterStatus = 0;
int   zaehler = 0;
bool  debug = true;

struct Auswertung {
  bool  hatFlaeche;
  float flaeche;
  float gutAnteil;
  float sig;
};

// ============================================================
//  Anzeige
// ============================================================
void zeige(const char *text) {
  display.setFont(nullptr);
  display.displayClear();
  display.print(text);
}

void zeigeBloecke(int b) {
  int mm = b * BLOCK_MM;                    // 0, 37, 74, 111 ...
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", mm / 10, mm % 10);
  display.setFont(zahlenFont);
  display.setCharSpacing(1);
  display.displayClear();
  display.print(text);
  Serial.printf(">> %d Bloecke = %s\n", b, text);
}

// ============================================================
//  Sensor
// ============================================================
void sortiere(int *w, int n) {
  for (int i = 1; i < n; i++) {
    int x = w[i], j = i - 1;
    while (j >= 0 && w[j] > x) { w[j + 1] = w[j]; j--; }
    w[j + 1] = x;
  }
}

bool leseSample() {
  if (!sensor.dataReady()) return false;
  int16_t d = sensor.distance();
  uint8_t st = 255;
  sensor.VL53L1X_GetRangeStatus(&st);
  uint16_t sg = 0;
  sensor.VL53L1X_GetSignalRate(&sg);
  sensor.clearInterrupt();
  letzterStatus = st;
  // 0 = gut, 1 = Sigma-Warnung, 2 = schwaches Signal (typisch bei blau)
  bool gut = (st == 0 || st == 1 || st == 2) && d > 0 && d < 2000;
  fDist[fIdx] = gut ? d : 0;
  fGut[fIdx] = gut;
  fSig[fIdx] = sg;
  fIdx = (fIdx + 1) % N_FENSTER;
  if (fN < N_FENSTER) fN++;
  return true;
}

Auswertung auswerten() {
  Auswertung a;
  a.hatFlaeche = false;
  a.flaeche = 0;
  int s[N_FENSTER];
  int cnt = 0;
  long sigSumme = 0;
  for (int i = 0; i < fN; i++) {
    if (fGut[i]) s[cnt++] = fDist[i];
    sigSumme += fSig[i];
  }
  a.gutAnteil = fN ? (float)cnt / fN : 0;
  a.sig = fN ? (float)sigSumme / fN : 0;
  if (cnt >= MIN_TREFFER) {
    sortiere(s, cnt);
    for (int i = 0; i + MIN_TREFFER <= cnt; i++) {      // NAECHSTE Flaeche mit genug Treffern
      long summe = 0;
      int j = i;
      while (j < cnt && s[j] - s[i] <= CLUSTER_BREITE) { summe += s[j]; j++; }
      if (j - i >= MIN_TREFFER) { a.hatFlaeche = true; a.flaeche = (float)summe / (j - i); break; }
    }
  }
  return a;
}

bool sensorStarten() {
  for (int versuch = 1; versuch <= 5; versuch++) {
    if (sensor.begin(0x29, &Wire)) {
      sensor.VL53L1X_SetDistanceMode(1);
      sensor.VL53L1X_SetROI(ROI_GROESSE, ROI_GROESSE);
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
//  Nullpunkt (Tisch leer!)
// ============================================================
bool nullpunktSetzen() {
  zeige("0...");
  Serial.println("Nullpunkt: Tisch LEER lassen!");
  delay(500);
  fN = 0; fIdx = 0;
  int gesammelt = 0;
  unsigned long start = millis();
  while (gesammelt < 2 * N_FENSTER && millis() - start < 20000) {
    if (leseSample()) gesammelt++;
    delay(1);
  }
  Auswertung a = auswerten();
  if (!a.hatFlaeche || a.gutAnteil < 0.3f) return false;
  baseDist = a.flaeche;
  baseGut = a.gutAnteil;
  baseSig = a.sig;
  fN = 0; fIdx = 0;
  bloecke = 0; level = baseDist; anker = baseDist; stabil = 0;
  Serial.printf("Leer: Abstand %.1f mm | gueltig %.0f %% | Signal %.0f\n", baseDist, baseGut * 100, baseSig);
  zeigeBloecke(0);
  return true;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'k') { if (!nullpunktSetzen()) zeige("ERR"); }
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
  if (!nullpunktSetzen()) {
    Serial.println("Nullpunkt fehlgeschlagen - Sensor/Tisch pruefen.");
    zeige("ERR");
    while (true) delay(1000);
  }
  Serial.println("Bereit. Bloecke einzeln auflegen. (k = Nullpunkt, d = Debug)");
}

void loop() {
  seriellBefehle();
  if (!leseSample()) return;
  if (fN < N_FENSTER) return;

  Auswertung a = auswerten();

  // Effektiver Abstand: A) Abstand, sonst B/C) dunkler Block, sonst leer
  bool blockAbstand = a.hatFlaeche && (baseDist - a.flaeche >= STUFE_MIN_MM);
  bool dunkel = (baseSig > 20 && a.sig < DUNKEL_SIGNAL * baseSig) ||
                (a.gutAnteil < baseGut - DUNKEL_GUELTIG);
  float eff = baseDist;
  if (blockAbstand)      eff = a.flaeche;
  else if (dunkel)       eff = baseDist - BLOCK_MM;      // dunkler Block, Abstand unbekannt

  // Ruhig?
  if (fabsf(eff - anker) <= STABIL_TOL_MM) {
    if (stabil < 1000) stabil++;
  } else if (stabil > 2) {
    stabil -= 2;                                          // einzelne Ausreisser verzeihen
  } else {
    anker = eff;
    stabil = 0;
  }

  // Einmal pro ruhigem Zustand auswerten: jeder erkannte Block = +1
  if (stabil == STABIL_NOETIG) {
    float diff = level - anker;                           // > 0: naeher = Block dazu
    int alt = bloecke;
    if (fabsf(anker - baseDist) < STUFE_MIN_MM) {         // wieder leer
      bloecke = 0;
      level = baseDist;
    } else if (diff >= STUFE_MIN_MM) {
      int n = (int)lroundf(diff / BLOCK_MM);
      if (n < 1) n = 1;
      bloecke += n;
      level = anker;
    } else if (diff <= -STUFE_MIN_MM) {
      int n = (int)lroundf(-diff / BLOCK_MM);
      if (n < 1) n = 1;
      bloecke -= n;
      level = anker;
    }
    if (bloecke < 0) bloecke = 0;
    if (bloecke > MAX_BLOECKE) bloecke = MAX_BLOECKE;
    if (bloecke != alt) zeigeBloecke(bloecke);
  }

  if (debug && (++zaehler % 5 == 0)) {
    Serial.printf("Flaeche: %6.1f | gueltig: %3.0f%% | Signal: %5.0f (leer %5.0f) | %s | ruhig: %2d | Status: %d | -> %d Bloecke\n",
                  a.hatFlaeche ? a.flaeche : 0.0f, a.gutAnteil * 100, a.sig, baseSig,
                  blockAbstand ? "ABSTAND" : (dunkel ? "DUNKEL " : "leer   "),
                  stabil, letzterStatus, bloecke);
  }
}
