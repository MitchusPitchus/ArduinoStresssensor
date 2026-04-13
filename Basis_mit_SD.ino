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

// Klassifikation für eine Achse
StressLevel classifyAxis(int16_t value,
                         int16_t uMin, int16_t uMax,
                         int16_t sMin, int16_t sMax)
{
  if (value >= uMin && value <= uMax) {
    return STRESS_UNGESTRESST;
  }

  if ((value >= sMin && value < uMin) || (value > uMax && value <= sMax)) {
    return STRESS_LEICHT;
  }

  if (value < sMin || value > sMax) {
    return STRESS_GESTRESST;
  }

  return STRESS_GESTRESST;
}

// Kombiniere Gx, Gy, Gz zu einem Gesamt-Stresslevel
StressLevel calcOverallStress(const Payload &p) {
  StressLevel gxLvl = classifyAxis(p.gx, GX_UN_MIN, GX_UN_MAX, GX_ST_MIN, GX_ST_MAX);
  StressLevel gyLvl = classifyAxis(p.gy, GY_UN_MIN, GY_UN_MAX, GY_ST_MIN, GY_ST_MAX);
  StressLevel gzLvl = classifyAxis(p.gz, GZ_UN_MIN, GZ_UN_MAX, GZ_ST_MIN, GZ_ST_MAX);

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
    case STRESS_UNGESTRESST: return F("UNGESTRESST");
    case STRESS_LEICHT:      return F("LEICHT");
    case STRESS_GESTRESST:   return F("GESTRESST");
    default:                 return F("?");
  }
}

void updateTrafficLight() {
  uint8_t stressedCount = 0;
  uint8_t lightStressCount = 0;

  for (uint8_t id = 1; id <= NUM_NODES; id++) {
    if (lastSeen[id] > 0) {
      StressLevel lvl = calcOverallStress(lastPayload[id]);

      if (lvl == STRESS_GESTRESST) {
        stressedCount++;
      } else if (lvl == STRESS_LEICHT) {
        lightStressCount++;
      }
    }
  }

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  if (stressedCount >= 2) {
    digitalWrite(LED_RED_PIN, HIGH);
  } 
  else if (stressedCount >= 1 || lightStressCount >= 2) {
    digitalWrite(LED_YELLOW_PIN, HIGH);
  } 
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