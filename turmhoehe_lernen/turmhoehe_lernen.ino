// Turmhoehe mit gelernter Stufen-Tabelle (weisse UND blaue Bloecke)
// 0 Bloecke = 0.0cm, jeder erkannte Block = +3.7cm.
// Serial (115200): l = Tabelle | x = loeschen + neu nullen | k = Nullpunkt (Tisch leer!) | d = Debug

#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
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
// Blaue Bloecke werfen weniger Infrarot zurueck -> lange Messzeit, niedrige Signal-Schwelle,
// und Status 2 ("Signal schwach") wird NICHT mehr weggeworfen.
#define TIMING_BUDGET_MS  100
#define ROI_GROESSE       6      // 4 = schmaler (weniger Signal), 6 = Kompromiss
#define SIGNAL_SCHWELLE   300    // kcps (Standard 1024)
#define SIGMA_SCHWELLE    30     // mm   (Standard 15)

// ---------------- BLOECKE -------------------
#define BLOCK_ANZEIGE_MM  37     // 3,7 cm pro Block
#define MAX_BLOECKE       17

// ---------------- MESSUNG -------------------
#define FILTER_N       24        // Messwerte im Fenster (~2,4 s)
#define CLUSTER_BREITE 10.0f     // mm: Werte innerhalb gelten als dieselbe Flaeche
#define MIN_TREFFER    2         // so viele Messungen muessen die naechste Flaeche bestaetigen
#define STABIL_TOL_MM  5.0f      // Abstand gilt als ruhig, solange er sich nicht mehr aendert
#define STABIL_NOETIG  3         // ruhige Auswertungen bevor die Anzeige wechselt
#define NEU_STABIL     8         // ruhige Auswertungen bevor ein NEUER Block gelernt wird (~0,8 s)
#define MIN_STUFE      8         // mm: kleinste Aenderung, die als Block gilt

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

// --------------------------------------------
Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
Preferences speicher;

// stufe[n] = gemessener Abstand (mm) bei n Bloecken. stufe[0] = leerer Boden.
int16_t stufe[MAX_BLOECKE + 1];
int stufenAnzahl = 0;

int puffer[FILTER_N];
int pIndex = 0;
int pAnzahl = 0;

int angezeigt = -1;
int ruhigZaehler = 0;
float anker = 0;                 // Abstand, auf dem der Sensor gerade ruhig steht
uint8_t letzterStatus = 0;
bool debug = true;

// ============================================================
//  Anzeige
// ============================================================
void zeige(const char *text) {
  display.setFont(nullptr);
  display.displayClear();
  display.print(text);
}

void zeigeBloecke(int bloecke) {
  int mm = bloecke * BLOCK_ANZEIGE_MM;      // 0, 37, 74, 111 ...
  char text[12];
  snprintf(text, sizeof(text), "%d.%dcm", mm / 10, mm % 10);
  display.setFont(zahlenFont);
  display.setCharSpacing(1);
  display.displayClear();
  display.print(text);
  Serial.printf(">> %d Bloecke = %s\n", bloecke, text);
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

bool leseAbstand(int &mm) {
  if (!sensor.dataReady()) return false;
  int16_t d = sensor.distance();
  uint8_t status = 255;
  sensor.VL53L1X_GetRangeStatus(&status);
  sensor.clearInterrupt();
  letzterStatus = status;
  // 0 = gut, 1 = Sigma-Warnung, 2 = schwaches Signal (typisch bei BLAU) -> alle drei gelten
  bool ok = (status == 0 || status == 1 || status == 2);
  if (!ok || d <= 0 || d > 2000) return false;
  mm = d;
  return true;
}

// Der Sensor sieht Block UND Tisch gleichzeitig; die Werte springen dazwischen.
// Darum zaehlt die NAECHSTE Flaeche, die von genug Messungen bestaetigt wird.
bool naechsteFlaeche(const int *werte, int n, int noetig, float &ergebnis) {
  int s[FILTER_N > 60 ? FILTER_N : 60];
  if (n > 60) n = 60;
  for (int i = 0; i < n; i++) s[i] = werte[i];
  sortiere(s, n);
  for (int i = 0; i + noetig <= n; i++) {
    long summe = 0;
    int j = i;
    while (j < n && s[j] - s[i] <= CLUSTER_BREITE) { summe += s[j]; j++; }
    if (j - i >= noetig) { ergebnis = (float)summe / (j - i); return true; }
  }
  return false;
}

// 60 Messungen, obere/untere 25 % weg, Rest mitteln (leerer Tisch)
int messeGenau() {
  const int N = 60;
  int w[N];
  int ok = 0;
  unsigned long start = millis();
  while (ok < N && millis() - start < 15000) {
    int d;
    if (leseAbstand(d)) w[ok++] = d;
    delay(1);
  }
  if (ok < 20) return -1;
  sortiere(w, ok);
  long summe = 0;
  for (int i = ok / 4; i < ok - ok / 4; i++) summe += w[i];
  return (int)(summe / (ok - 2 * (ok / 4)));
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
//  Stufen-Tabelle
// ============================================================
void tabelleSpeichern() {
  speicher.putInt("anz", stufenAnzahl);
  speicher.putBytes("tab", stufe, sizeof(stufe));
}

void tabelleLaden() {
  stufenAnzahl = speicher.getInt("anz", 0);
  if (stufenAnzahl < 1 || stufenAnzahl > MAX_BLOECKE + 1 ||
      speicher.getBytes("tab", stufe, sizeof(stufe)) != sizeof(stufe)) {
    stufenAnzahl = 0;
  }
}

void tabelleZeigen() {
  Serial.println("Stufen-Tabelle:");
  for (int i = 0; i < stufenAnzahl; i++) {
    Serial.printf("  %2d Bloecke: %4d mm", i, stufe[i]);
    if (i > 0) Serial.printf("   (Block %d misst %d mm)", i, stufe[i - 1] - stufe[i]);
    Serial.println();
  }
  if (stufenAnzahl <= 1) Serial.println("  (noch keine Bloecke gelernt - einfach Block fuer Block auflegen)");
}

float naechsteStufeMM() {
  if (stufenAnzahl < 2) return 0;
  int L = stufenAnzahl - 1;
  return stufe[L - 1] - stufe[L];
}

// Abstand -> Blockanzahl. Darf die Tabelle erweitern/korrigieren. -1 = noch abwarten.
int bestimmeBloecke(float a) {
  int L = stufenAnzahl - 1;

  // 1) Hoeher als alle bekannten Stufen? -> GENAU 1 Block mehr
  float luecke = max((float)MIN_STUFE, naechsteStufeMM() * 0.5f);
  if (a < stufe[L] - luecke) {
    if (ruhigZaehler < NEU_STABIL) return -1;
    if (stufenAnzahl > MAX_BLOECKE) return L;
    stufe[stufenAnzahl] = (int16_t)lroundf(a);
    stufenAnzahl++;
    tabelleSpeichern();
    Serial.printf("Neuer Block gelernt: %d Bloecke = %d mm\n", stufenAnzahl - 1, stufe[stufenAnzahl - 1]);
    return stufenAnzahl - 1;
  }

  // 2) Naechstgelegene bekannte Stufe
  int best = 0;
  float bestAbst = 1e9;
  for (int i = 0; i < stufenAnzahl; i++) {
    float dd = fabsf(a - stufe[i]);
    if (dd < bestAbst) { bestAbst = dd; best = i; }
  }

  // 3) Selbstkorrektur: liegt der Wert klar ZWISCHEN zwei Stufen
  int oben = (a < stufe[best]) ? best + 1 : best;
  if (oben >= 1 && oben < stufenAnzahl) {
    float schritt = stufe[oben - 1] - stufe[oben];
    float abstUnten = stufe[oben - 1] - a;
    float abstOben  = a - stufe[oben];
    float grenze = max(6.0f, schritt * 0.35f);
    if (abstUnten > grenze && abstOben > grenze && ruhigZaehler >= NEU_STABIL) {
      stufe[oben] = (int16_t)lroundf(a);
      stufenAnzahl = oben + 1;
      tabelleSpeichern();
      Serial.printf("Stufe %d korrigiert: %d mm (Stufen darueber geloescht, werden neu gelernt)\n",
                    oben, stufe[oben]);
      return oben;
    }
  }

  // 4) Hysterese
  if (angezeigt >= 0 && angezeigt < stufenAnzahl &&
      fabsf(a - stufe[angezeigt]) < bestAbst + 3.0f) {
    return angezeigt;
  }
  return best;
}

bool nullpunktSetzen() {
  zeige("0...");
  Serial.println("Nullpunkt: nichts unter den Sensor stellen!");
  delay(300);
  int d = messeGenau();
  if (d < 0) return false;

  if (stufenAnzahl >= 2) {
    int abw = d - stufe[0];
    if (abs(abw) <= max(4, (int)(stufe[0] - stufe[1]) / 3)) {
      for (int i = 0; i < stufenAnzahl; i++) stufe[i] += abw;
      Serial.printf("Tabelle um %d mm angepasst\n", abw);
    } else {
      Serial.printf("Boden passt nicht zur alten Tabelle (%d statt %d mm) -> Tabelle neu\n", d, stufe[0]);
      stufenAnzahl = 1;
    }
  } else {
    stufenAnzahl = 1;
  }
  stufe[0] = d;
  tabelleSpeichern();

  Serial.printf("Boden-Abstand: %d mm\n", d);
  pAnzahl = 0;
  ruhigZaehler = 0;
  anker = d;
  angezeigt = 0;
  zeigeBloecke(0);
  return true;
}

void seriellBefehle() {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'l') tabelleZeigen();
    else if (c == 'x') {
      stufenAnzahl = 1;
      tabelleSpeichern();
      Serial.println("Tabelle geloescht. Nullpunkt wird neu gemessen.");
      nullpunktSetzen();
    }
    else if (c == 'k') { if (!nullpunktSetzen()) zeige("ERR"); }
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

  speicher.begin("turm4", false);   // neuer Speicher -> alte, evtl. falsche Tabelle wird nicht benutzt
  tabelleLaden();

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

  tabelleZeigen();
  Serial.println("Bereit. Bloecke einzeln auflegen. (l = Tabelle, x = loeschen, k = Nullpunkt, d = Debug)");
}

void loop() {
  seriellBefehle();

  int d;
  if (!leseAbstand(d)) return;

  puffer[pIndex] = d;
  pIndex = (pIndex + 1) % FILTER_N;
  if (pAnzahl < FILTER_N) pAnzahl++;
  if (pAnzahl < FILTER_N) return;

  float abstand;
  if (!naechsteFlaeche(puffer, FILTER_N, MIN_TREFFER, abstand)) return;

  // Ruhig = der Abstand bleibt auf derselben Flaeche. Einzelne Ausreisser (Sensor sieht kurz
  // den Tisch statt des Blocks) setzen den Zaehler nur etwas zurueck, nicht auf 0.
  if (fabsf(abstand - anker) <= STABIL_TOL_MM) {
    if (ruhigZaehler < 1000) ruhigZaehler++;
  } else if (ruhigZaehler > 2) {
    ruhigZaehler -= 2;
  } else {
    anker = abstand;
    ruhigZaehler = 0;
  }

  int neu = bestimmeBloecke(anker);

  if (debug) {
    Serial.printf("roh: %4d | Flaeche: %6.1f | Stand: %6.1f | ruhig: %2d | Status: %d | -> %d Bloecke\n",
                  d, abstand, anker, ruhigZaehler, letzterStatus, neu);
  }

  if (neu >= 0 && neu != angezeigt && ruhigZaehler >= STABIL_NOETIG) {
    angezeigt = neu;
    zeigeBloecke(neu);
  }
}
