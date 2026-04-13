#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

RF24 radio(9, 10);                  // CE, CSN
const byte address[6] = "NOD01";

// ---------------- SD ----------------
const uint8_t SD_CS_PIN = 8;        // SD-CS an D8
bool sdOk = false;
File logFile;

// ---------------- Ampel ----------------
const uint8_t LED_RED_PIN    = 5;
const uint8_t LED_YELLOW_PIN = 4;
const uint8_t LED_GREEN_PIN  = 3;

// ---------------- Payload ----------------
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

// --------------------------------------------------
// Stress-Grenzwerte Phase B (Inspire Living Lab UMM)
// 0 = nicht gestresst
// 2 = leicht belastet
// 4 = leicht gestresst
// 8 = (stark) gestresst
// --------------------------------------------------

// Stresslevel 0
const int16_t GX_0_MIN = -4;   // aus -3.75 gerundet
const int16_t GX_0_MAX =  7;   // aus  7.25 gerundet
const int16_t GY_0_MIN = -6;   // aus -5.5 gerundet
const int16_t GY_0_MAX =  9;
const int16_t GZ_0_MIN = -7;   // aus -6.5 gerundet
const int16_t GZ_0_MAX =  4;

// Stresslevel 2
const int16_t GX_2_MIN = -31;
const int16_t GX_2_MAX =  24;  // aus 24.2 gerundet
const int16_t GY_2_MIN = -67;
const int16_t GY_2_MAX =  76;  // aus 75.8 gerundet
const int16_t GZ_2_MIN = -18;
const int16_t GZ_2_MAX =  19;  // aus 18.6 gerundet

// Stresslevel 4
const int16_t GX_4_MIN = -58;  // aus -58.25 gerundet
const int16_t GX_4_MAX =  41;  // aus 41.15 gerundet
const int16_t GY_4_MIN = -129; // aus -128.9 gerundet
const int16_t GY_4_MAX =  143; // aus 142.6 gerundet
const int16_t GZ_4_MIN = -30;  // aus -29.5 gerundet
const int16_t GZ_4_MAX =  33;  // aus 33.2 gerundet

// Stresslevel 8
const int16_t GX_8_MIN = -113; // aus -112.75 gerundet
const int16_t GX_8_MAX =   75; // aus 75.05 gerundet
const int16_t GY_8_MIN = -252; // aus -252.3 gerundet
const int16_t GY_8_MAX =  276; // aus 276.2 gerundet
const int16_t GZ_8_MIN =  -53; // aus -52.5 gerundet
const int16_t GZ_8_MAX =   62; // aus 62.4 gerundet

// Stress-Level Enum
enum StressLevel : uint8_t {
  STRESS_0_NICHT          = 0,
  STRESS_2_LEICHT_BELASTET = 1,
  STRESS_4_LEICHT_GESTRESST = 2,
  STRESS_8_GESTRESST      = 3
};

bool initSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  // WICHTIG für Mega: Hardware-SS als Output
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD-Init fehlgeschlagen!"));
    return false;
  }

  Serial.println(F("SD-Karte bereit."));

  bool writeHeader = !SD.exists("log.csv");

  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println(F("Konnte log.csv nicht oeffnen!"));
    return false;
  }

  if (writeHeader) {
    logFile.println(F("millis,id,rf,stress,ax,ay,az,gx,gy,gz,pitch,roll,yaw"));
    logFile.flush();
  }

  logFile.close();
  return true;
}

// Klassifikation für eine Achse nach 4 Stufen
StressLevel classifyAxis(int16_t value,
                         int16_t l0Min, int16_t l0Max,
                         int16_t l2Min, int16_t l2Max,
                         int16_t l4Min, int16_t l4Max,
                         int16_t l8Min, int16_t l8Max)
{
  if (value >= l0Min && value <= l0Max) {
    return STRESS_0_NICHT;
  }

  if (value >= l2Min && value <= l2Max) {
    return STRESS_2_LEICHT_BELASTET;
  }

  if (value >= l4Min && value <= l4Max) {
    return STRESS_4_LEICHT_GESTRESST;
  }

  if (value >= l8Min && value <= l8Max) {
    return STRESS_8_GESTRESST;
  }

  // Alles außerhalb des 8er Bereichs wird als stark gestresst gewertet
  return STRESS_8_GESTRESST;
}

// Kombiniere Gx, Gy, Gz zu einem Gesamt-Stresslevel
StressLevel calcOverallStress(const Payload &p) {
  StressLevel gxLvl = classifyAxis(p.gx,
                                   GX_0_MIN, GX_0_MAX,
                                   GX_2_MIN, GX_2_MAX,
                                   GX_4_MIN, GX_4_MAX,
                                   GX_8_MIN, GX_8_MAX);

  StressLevel gyLvl = classifyAxis(p.gy,
                                   GY_0_MIN, GY_0_MAX,
                                   GY_2_MIN, GY_2_MAX,
                                   GY_4_MIN, GY_4_MAX,
                                   GY_8_MIN, GY_8_MAX);

  StressLevel gzLvl = classifyAxis(p.gz,
                                   GZ_0_MIN, GZ_0_MAX,
                                   GZ_2_MIN, GZ_2_MAX,
                                   GZ_4_MIN, GZ_4_MAX,
                                   GZ_8_MIN, GZ_8_MAX);

  // Höchstes Stresslevel gewinnt
  StressLevel overall = gxLvl;
  if (gyLvl > overall) overall = gyLvl;
  if (gzLvl > overall) overall = gzLvl;

  return overall;
}

const __FlashStringHelper* stressToText(StressLevel lvl) {
  switch (lvl) {
    case STRESS_0_NICHT:             return F("NICHT");
    case STRESS_2_LEICHT_BELASTET:   return F("BELASTET");
    case STRESS_4_LEICHT_GESTRESST:  return F("LEICHT GESTR.");
    case STRESS_8_GESTRESST:         return F("GESTRESST");
    default:                         return F("?");
  }
}

void updateTrafficLight() {
  uint8_t stressedCount = 0;
  uint8_t lightStressedCount = 0;
  uint8_t burdenedCount = 0;

  for (uint8_t id = 1; id <= NUM_NODES; id++) {
    if (lastSeen[id] > 0) {
      StressLevel lvl = calcOverallStress(lastPayload[id]);

      if (lvl == STRESS_8_GESTRESST) {
        stressedCount++;
      } else if (lvl == STRESS_4_LEICHT_GESTRESST) {
        lightStressedCount++;
      } else if (lvl == STRESS_2_LEICHT_BELASTET) {
        burdenedCount++;
      }
    }
  }

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  // Rot: mindestens 2 stark gestresst ODER 1 stark + 1 leicht gestresst
  if (stressedCount >= 2 || (stressedCount >= 1 && lightStressedCount >= 1)) {
    digitalWrite(LED_RED_PIN, HIGH);
  }
  // Gelb: mindestens 1 stark gestresst ODER 2 leicht gestresst ODER mehrere belastet
  else if (stressedCount >= 1 || lightStressedCount >= 2 || burdenedCount >= 2) {
    digitalWrite(LED_YELLOW_PIN, HIGH);
  }
  // Sonst grün
  else {
    digitalWrite(LED_GREEN_PIN, HIGH);
  }
}

void logToSD(uint8_t id, bool rf_ok, const Payload &p) {
  if (!sdOk) return;

  logFile = SD.open("log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println(F("Fehler: log.csv konnte nicht geoeffnet werden"));
    return;
  }

  StressLevel lvl = calcOverallStress(p);

  logFile.print(millis());
  logFile.print(',');

  logFile.print(id);
  logFile.print(',');

  logFile.print(rf_ok ? F("OK") : F("NOK"));
  logFile.print(',');

  logFile.print(stressToText(lvl));
  logFile.print(',');

  logFile.print(p.ax);    logFile.print(',');
  logFile.print(p.ay);    logFile.print(',');
  logFile.print(p.az);    logFile.print(',');
  logFile.print(p.gx);    logFile.print(',');
  logFile.print(p.gy);    logFile.print(',');
  logFile.print(p.gz);    logFile.print(',');
  logFile.print(p.pitch); logFile.print(',');
  logFile.print(p.roll);  logFile.print(',');
  logFile.println(p.yaw);

  logFile.flush();
  logFile.close();
}

void sendCSV(uint8_t id, bool rf_ok, const Payload &rx) {
  StressLevel lvl = calcOverallStress(rx);

  Serial.print(id); Serial.print(",");
  Serial.print(rf_ok ? "OK" : "NOK"); Serial.print(",");
  Serial.print(stressToText(lvl)); Serial.print(",");
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

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("ID: "));
  display.println(id);

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

  // WICHTIG für Mega-SPI
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  // Optional: SPI-Teilnehmer erstmal deaktivieren
  pinMode(8, OUTPUT);   // SD CS
  digitalWrite(8, HIGH);

  pinMode(10, OUTPUT);  // nRF24 CSN
  digitalWrite(10, HIGH);

  pinMode(9, OUTPUT);   // nRF24 CE
  digitalWrite(9, LOW);

  // --- Ampel Pins ---
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);

  // --- OLED initialisieren ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED nicht gefunden!"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("MEGA-Empfaenger"));
    display.println(F("Init RF24 + SD..."));
    display.display();
  }

  // Arrays initialisieren
  for (uint8_t i = 0; i <= NUM_NODES; i++) {
    lastSeen[i] = 0;
  }

  // --- SD initialisieren ---
  sdOk = initSD();

  // --- nRF24 ---
  radio.begin();
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(90);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println(F("MEGA-Empfaenger bereit. Warte auf Daten..."));

  currentNode = 1;
  currentPhase = 0;
  lastPhaseSwitch = millis();
}

void loop() {
  unsigned long now = millis();

  // --- Pakete empfangen & lastSeen / lastPayload updaten ---
  while (radio.available()) {
    radio.read(&rx, sizeof(rx));

    if (rx.id >= 1 && rx.id <= NUM_NODES) {
      lastSeen[rx.id] = now;
      lastPayload[rx.id] = rx;
    }

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
    logToSD(rx.id, true, rx);
  }

  // --- Phasenwechsel: 5s RF, 5s Stress ---
  if (now - lastPhaseSwitch >= PHASE_HOLD_TIME) {
    lastPhaseSwitch = now;
    currentPhase++;

    if (currentPhase > 1) {
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
      hasData = true;
      rf_ok = false;
    } else {
      hasData = false;
      rf_ok = false;
    }
  }

  // --- Anzeige aktualisieren ---
  static unsigned long lastDisplayUpdate = 0;
  if (now - lastDisplayUpdate > 200) {
    if (currentPhase == 0) {
      showRFStatusOnOLED(currentNode, rf_ok);
    } else {
      showStressOnOLED(currentNode, hasData, lastPayload[currentNode]);
    }
    lastDisplayUpdate = now;
  }

  // --- Ampel aktualisieren ---
  updateTrafficLight();

  delay(10);
}
