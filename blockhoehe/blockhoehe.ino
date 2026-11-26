// Blockhoehen-Messer: ESP32 + VL53L1X + 4x MAX7219 (FC16)
// Sensor schaut von oben auf die Tischflaeche.
//   - Kein Klotz  -> Anzeige 0.0
//   - Klotz/Turm  -> Anzeige = Hoehe in cm (Studs inklusive)
// Nullpunkt wird beim Start (leerer Tisch!) automatisch eingemessen.
// Neu nullen: BOOT-Taste (GPIO0) druecken oder 'z' im Serial Monitor senden.
// Serial-Befehle: 'z' nullen | 'b3.7' Blockhoehe in cm setzen | 'm' Anzeige Hoehe <-> Blockanzahl

#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <math.h>

// ---------- Hardware ----------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES   4
#define DATA_PIN      23
#define CS_PIN        5
#define CLK_PIN       18
#define SDA_PIN       21
#define SCL_PIN       22
#define ZERO_BUTTON   0      // BOOT-Taste

Adafruit_VL53L1X sensor;
MD_Parola display(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// ---------- Einstellungen ----------
const uint16_t TIMING_MS     = 50;    // Messzeit pro Einzelmessung (mehr = genauer)
const bool USE_NARROW_ROI    = true;  // engeres Blickfeld -> Klotz muss direkt unter dem Sensor stehen
const uint16_t ROI_SIZE      = 4;     // 4..16 (SPADs pro Seite), 4 = engstes Blickfeld

const int   SAMPLES          = 10;    // Einzelmessungen pro Anzeigewert (~0.5 s)
const int   CAL_SAMPLES      = 40;    // Einzelmessungen fuer den Nullpunkt
const float MAX_DEV_CM       = 0.4;   // Abweichung vom Median, ab der ein Wert Ausreisser ist
const float MIN_VALID_RATIO  = 0.7;   // Anteil gueltiger Einzelmessungen, sonst Messung verwerfen
const float CLUSTER_MIN_RATIO = 0.3;  // so viele Messungen muessen den naechsten Wert (Klotz) bestaetigen
const float ZERO_BAND_CM     = 0.2;   // darunter = "kein Klotz" -> 0.0
const float STEP_CM          = 0.4;   // groessere Aenderung = Klotz aufgelegt/entfernt -> sofort uebernehmen
const float SMOOTHING        = 0.5;   // Glaettung bei kleinen Aenderungen (0..1)
const float DISPLAY_HYST_CM  = 0.15;  // Anzeige aendert sich erst ab dieser Differenz (kein Flackern)
const float BLOCK_TOLERANCE  = 0.3;   // cm: so weit darf die Hoehe vom Vielfachen der Blockhoehe abweichen

// ---------- Zustand ----------
float blockCm    = 3.7;   // Hoehe eines Blocks (zur Laufzeit aenderbar: 'b3.5' im Serial Monitor)
bool  showBlocks = false; // 'm' im Serial Monitor: Anzeige Hoehe (cm) <-> Anzahl Bloecke
float baselineCm = 0.0;   // Abstand Sensor -> leerer Tisch
float smoothCm   = 0.0;
float shownCm    = -1.0;
int   failCount  = 0;

// ---------- Hilfsfunktionen ----------
void showText(const char *t) {
  display.print(t);
}

void showValue(float v) {
  char buf[12];
  dtostrf(v, 1, 1, buf);          // z.B. "0.0", "12.3"
  showText(buf);
}

void sortFloats(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float k = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; }
    a[j + 1] = k;
  }
}

// Liest n Einzelmessungen (cm). Gibt Anzahl gueltiger Werte zurueck.
int collect(float *out, int n) {
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    unsigned long t0 = millis();
    while (!sensor.dataReady()) {
      if (millis() - t0 > 500) return cnt;   // Sensor antwortet nicht
      delay(1);
    }
    int16_t mm = sensor.distance();
    sensor.clearInterrupt();
    if (mm > 0 && mm < 1300) out[cnt++] = mm / 10.0f;
  }
  return cnt;
}

// Robuster Abstand. Der Sensor sieht mit seinem Blickfeld oft Klotz UND Tisch zugleich.
// Darum: sortieren und die NAECHSTE Gruppe (kleinster Abstand) nehmen, die von genug
// Einzelmessungen bestaetigt wird. Rueckgabe -1 = unbrauchbar.
float measure(int n) {
  float s[CAL_SAMPLES > SAMPLES ? CAL_SAMPLES : SAMPLES];
  int cnt = collect(s, n);
  if (cnt < (int)(n * MIN_VALID_RATIO)) return -1;
  sortFloats(s, cnt);

  int need = (int)ceilf(cnt * CLUSTER_MIN_RATIO);
  if (need < 3) need = 3;
  for (int i = 0; i + need <= cnt; i++) {
    float sum = 0;
    int j = i;
    while (j < cnt && s[j] - s[i] <= 2.0f * MAX_DEV_CM) { sum += s[j]; j++; }
    if (j - i >= need) return sum / (j - i);
  }
  return -1;
}

// Nullpunkt einmessen (Tisch muss leer sein)
void calibrate() {
  showText("ZERO");
  Serial.println("Kalibriere Nullpunkt (Tisch leer lassen)...");
  for (int attempt = 0; attempt < 10; attempt++) {
    float d = measure(CAL_SAMPLES);
    if (d > 0) {
      baselineCm = d;
      smoothCm = 0.0;
      shownCm = -1.0;
      Serial.print("Nullpunkt: ");
      Serial.print(baselineCm, 2);
      Serial.println(" cm");
      return;
    }
    Serial.println("Kalibrierung instabil, neuer Versuch...");
  }
  showText("CAL?");
  Serial.println("Kalibrierung fehlgeschlagen - Sensor/Tisch pruefen. Taste druecken zum Wiederholen.");
  baselineCm = 0;
}

void handleInput() {
  bool zero = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') zero = true;
    else if (c == 'm') {
      showBlocks = !showBlocks;
      shownCm = -1.0;                       // Anzeige neu zeichnen
      Serial.println(showBlocks ? "Anzeige: Anzahl Bloecke" : "Anzeige: Hoehe in cm");
    } else if (c == 'b') {
      float b = Serial.parseFloat();        // z.B. b3.5
      if (b >= 0.5 && b <= 30.0) {
        blockCm = b;
        shownCm = -1.0;
        Serial.print("Blockhoehe: "); Serial.print(blockCm, 2); Serial.println(" cm");
      } else {
        Serial.println("Ungueltige Blockhoehe (0.5 .. 30 cm)");
      }
    }
  }
  if (digitalRead(ZERO_BUTTON) == LOW) {
    delay(30);
    if (digitalRead(ZERO_BUTTON) == LOW) zero = true;
  }
  if (zero) calibrate();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(ZERO_BUTTON, INPUT_PULLUP);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  display.begin();
  display.setIntensity(4);
  display.setTextAlignment(PA_CENTER);
  display.displayClear();

  if (!sensor.begin(0x29, &Wire)) {
    Serial.print("Sensor Error! Status: ");
    Serial.println(sensor.vl_status);
    showText("ERROR");
    while (1) delay(100);
  }

  sensor.VL53L1X_SetDistanceMode(1);               // Short: genauer & unempfindlicher gegen Licht (bis ~1.3 m)
  sensor.setTimingBudget(TIMING_MS);
  sensor.VL53L1X_SetInterMeasurementInMs(TIMING_MS + 5);
  if (USE_NARROW_ROI) sensor.VL53L1X_SetROI(ROI_SIZE, ROI_SIZE);

  if (!sensor.startRanging()) {
    Serial.print("startRanging Error! Status: ");
    Serial.println(sensor.vl_status);
    showText("ERROR");
    while (1) delay(100);
  }

  showText("START");
  delay(1000);
  calibrate();
  showValue(0.0);
  Serial.println("System Ready!");
}

// ---------- Loop ----------
void loop() {
  handleInput();
  if (baselineCm <= 0) { delay(100); return; }

  float d = measure(SAMPLES);
  if (d < 0) {
    // instabil (Klotz wird bewegt) oder Sensorfehler -> letzten Wert behalten
    if (++failCount >= 20) { showText("ERR"); shownCm = -1.0; }
    return;
  }
  failCount = 0;

  float h = baselineCm - d;                 // Hoehe ueber dem Tisch
  if (h < ZERO_BAND_CM) h = 0.0;            // kein Klotz

  // grosse Spruenge (Klotz drauf/runter) sofort, kleine Aenderungen geglaettet
  if (fabsf(h - smoothCm) > STEP_CM) smoothCm = h;
  else                               smoothCm += SMOOTHING * (h - smoothCm);
  if (h == 0.0 && smoothCm < ZERO_BAND_CM) smoothCm = 0.0;

  // Anzeige nur aktualisieren, wenn sich der Wert wirklich geaendert hat
  bool toggledZero = (smoothCm == 0.0) != (shownCm == 0.0);
  if (shownCm < 0 || toggledZero || fabsf(smoothCm - shownCm) >= DISPLAY_HYST_CM) {
    shownCm = smoothCm;
    if (showBlocks) {
      char b[8];
      snprintf(b, sizeof(b), "%d", (int)roundf(shownCm / blockCm));
      showText(b);
    } else {
      showValue(shownCm);
    }
  }

  // Anzahl Bloecke und Plausibilitaet (Vielfaches der Blockhoehe?)
  int   blocks = (int)roundf(smoothCm / blockCm);
  float rest   = smoothCm - blocks * blockCm;
  bool  fits   = fabsf(rest) <= BLOCK_TOLERANCE;

  Serial.print("Abstand: ");   Serial.print(d, 2);
  Serial.print(" cm | Hoehe: "); Serial.print(h, 2);
  Serial.print(" cm | Anzeige: "); Serial.print(shownCm, 1);
  Serial.print(" cm | Bloecke: "); Serial.print(blocks);
  Serial.println(fits ? "" : " (Hoehe passt nicht zu ganzen Bloecken)");
}
