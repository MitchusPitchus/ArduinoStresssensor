#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

RF24 radio(9, 10);                  // CE, CSN
const byte address[6] = "NOD01";

struct Payload {
  uint8_t id;  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t pitch, roll, yaw;  // *100
} rx;

// --- OLED Setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Anzahl der erwarteten Sender
const uint8_t NUM_NODES = 3;   // IDs: 1, 2, 3

// Letzte Empfangszeit pro ID (Index = ID)
unsigned long lastSeen[NUM_NODES + 1];  // Index 0 ungenutzt

// Letzter Payload pro ID (Index = ID)
Payload lastPayload[NUM_NODES + 1];

// Timeout, ab wann ein Sender als NOK gilt
const unsigned long RF_TIMEOUT = 300;   // ms ohne Paket => RF: NOK

// Anzeige-Steuerung
uint8_t currentNode = 1;
unsigned long lastPhaseSwitch = 0;
const unsigned long PHASE_HOLD_TIME = 5000;  // 5 Sekunden pro Phase

// Phase 0 = RF-Status, Phase 1 = Stresslevel
uint8_t currentPhase = 0;

// --- Stress-Grenzwerte ---
// Ungestresst
const int16_t GX_UN_MIN = -81;
const int16_t GX_UN_MAX =  26;
const int16_t GY_UN_MIN = -70;
const int16_t GY_UN_MAX =  66;
const int16_t GZ_UN_MIN = -18;
const int16_t GZ_UN_MAX =  15;

// Gestresst
const int16_t GX_ST_MIN = -42;
const int16_t GX_ST_MAX =  53;
const int16_t GY_ST_MIN = -89;
const int16_t GY_ST_MAX = 134;
const int16_t GZ_ST_MIN = -35;
const int16_t GZ_ST_MAX =  33;

// Stress-Level Enum
enum StressLevel : uint8_t {
  STRESS_UNGESTRESST = 0,
  STRESS_LEICHT      = 1,
  STRESS_GESTRESST   = 2
};

void sendCSV(uint8_t id, bool rf_ok, Payload rx) {
  Serial.print(id); Serial.print(",");
  Serial.print(rf_ok ? "OK" : "NOK"); Serial.print(",");
  Serial.print(rx.ax); Serial.print(",");
  Serial.print(rx.ay); Serial.print(",");
  Serial.print(rx.az); Serial.print(",");
  Serial.print(rx.gx); Serial.print(",");
  Serial.print(rx.gy); Serial.print(",");
  Serial.print(rx.gz); Serial.print(",");
  Serial.print(rx.pitch); Serial.print(",");
  Serial.print(rx.roll); Serial.print(",");
  Serial.println(rx.yaw);
}

void showRFStatusOnOLED(uint8_t id, bool rf_ok) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Sender-ID (groß)
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("ID: "));
  display.println(id);

  // RF-Status
  display.setTextSize(2);
  display.setCursor(0, 32);
  display.print(F("RF: "));
  if (rf_ok) {
    display.print(F("OK"));
  } else {
    display.print(F("NOK"));
  }

  display.display();
}

// Klassifikation für eine Achse
StressLevel classifyAxis(int16_t value,
                         int16_t uMin, int16_t uMax,
                         int16_t sMin, int16_t sMax)
{
  // Annahme: sMin < uMin < uMax < sMax
  if (value >= uMin && value <= uMax) {
    return STRESS_UNGESTRESST;
  }

  if ((value >= sMin && value < uMin) || (value > uMax && value <= sMax)) {
    return STRESS_LEICHT;
  }

  if (value < sMin || value > sMax) {
    return STRESS_GESTRESST;
  }

  return STRESS_GESTRESST; // Fallback
}

// Kombiniere Gx, Gy, Gz zu einem Gesamt-Stresslevel
StressLevel calcOverallStress(const Payload &p) {
  StressLevel gxLvl = classifyAxis(p.gx, GX_UN_MIN, GX_UN_MAX, GX_ST_MIN, GX_ST_MAX);
  StressLevel gyLvl = classifyAxis(p.gy, GY_UN_MIN, GY_UN_MAX, GY_ST_MIN, GY_ST_MAX);
  StressLevel gzLvl = classifyAxis(p.gz, GZ_UN_MIN, GZ_UN_MAX, GZ_ST_MIN, GZ_ST_MAX);

  // Höchster Level gewinnt
  if (gxLvl == STRESS_GESTRESST || gyLvl == STRESS_GESTRESST || gzLvl == STRESS_GESTRESST) {
    return STRESS_GESTRESST;
  }
  if (gxLvl == STRESS_LEICHT || gyLvl == STRESS_LEICHT || gzLvl == STRESS_LEICHT) {
    return STRESS_LEICHT;
  }
  return STRESS_UNGESTRESST;
}

const __FlashStringHelper* stressToText(StressLevel lvl) {
  switch (lvl) {
    case STRESS_UNGESTRESST: return F("UNGESTR.");
    case STRESS_LEICHT:      return F("LEICHT");
    case STRESS_GESTRESST:   return F("GESTR.");
    default:                 return F("?");
  }
}

void showStressOnOLED(uint8_t id, bool hasData, const Payload &p) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("ID: "));
  display.println(id);

  display.setTextSize(1);
  display.setCursor(0, 24);
  display.println(F("STRESSLEVEL:"));

  display.setTextSize(2);
  display.setCursor(0, 40);

  if (!hasData) {
    display.print(F("KEINE DATEN"));
  } else {
    StressLevel lvl = calcOverallStress(p);
    display.print(stressToText(lvl));
  }

  display.display();
}

void setup() {
  Serial.begin(115200);

  // --- OLED initialisieren ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // ggf. Adresse pruefen
    Serial.println(F("OLED nicht gefunden!"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("UNO-Empfaenger"));
    display.println(F("Init RF24..."));
    display.display();
  }

  // Arrays initialisieren
  for (uint8_t i = 0; i <= NUM_NODES; i++) {
    lastSeen[i] = 0;
  }

  // --- nRF24 ---
  radio.begin();
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(90);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println(F("UNO-Empfaenger bereit. Warte auf Daten..."));

  currentNode = 1;
  currentPhase = 0; // Start mit RF-Anzeige
  lastPhaseSwitch = millis();
}

void loop() {
  unsigned long now = millis();

  // --- Pakete empfangen & lastSeen / lastPayload updaten ---
  while (radio.available()) {
    radio.read(&rx, sizeof(rx));

    // Nur IDs im erwarteten Bereich berücksichtigen
    if (rx.id >= 1 && rx.id <= NUM_NODES) {
      lastSeen[rx.id] = now;
      lastPayload[rx.id] = rx;  // letzten Payload speichern
    }

    // Optional: Debug auf Serial
    Serial.print(F("Von ID "));
    Serial.print(rx.id);

    Serial.print(F(" | AX: ")); Serial.print(rx.ax);
    Serial.print(F(" AY: "));  Serial.print(rx.ay);
    Serial.print(F(" AZ: "));  Serial.print(rx.az);

    Serial.print(F("  GX: ")); Serial.print(rx.gx);
    Serial.print(F(" GY: "));  Serial.print(rx.gy);
    Serial.print(F(" GZ: "));  Serial.print(rx.gz);

    Serial.print(F("  Pitch: ")); Serial.print(rx.pitch / 100.0f);
    Serial.print(F("  Roll: "));  Serial.print(rx.roll  / 100.0f);
    Serial.print(F("  Yaw: "));   Serial.println(rx.yaw  / 100.0f);

    sendCSV(rx.id, true, rx);
  }

  // --- Phasenwechsel: 5s RF, 5s Stress ---
  if (now - lastPhaseSwitch >= PHASE_HOLD_TIME) {
    lastPhaseSwitch = now;
    currentPhase++;

    if (currentPhase > 1) { // zurück zu RF und nächsten Node wählen
      currentPhase = 0;
      currentNode++;
      if (currentNode > NUM_NODES) {
        currentNode = 1;
      }
    }
  }

  // --- RF-Status für aktuell angezeigten Sender bestimmen ---
  bool rf_ok = false;
  bool hasData = false;

  if (currentNode >= 1 && currentNode <= NUM_NODES) {
    if (lastSeen[currentNode] > 0 && (now - lastSeen[currentNode] <= RF_TIMEOUT)) {
      rf_ok = true;
      hasData = true;
    } else if (lastSeen[currentNode] > 0) {
      // Es gab schon einmal Daten, aber aktuell Timeout:
      hasData = true;
      rf_ok = false;
    } else {
      // Noch nie Daten erhalten
      hasData = false;
      rf_ok = false;
    }
  }

  // --- Anzeige aktualisieren ---
  static unsigned long lastDisplayUpdate = 0;
  if (now - lastDisplayUpdate > 200) {  // alle 200 ms Display neu zeichnen
    if (currentPhase == 0) {
      // RF-Status-Phase
      showRFStatusOnOLED(currentNode, rf_ok);
    } else {
      // Stress-Phase
      showStressOnOLED(currentNode, hasData, lastPayload[currentNode]);
    }
    lastDisplayUpdate = now;
  }

  delay(10);
}
