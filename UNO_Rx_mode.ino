
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

// Timeout, ab wann ein Sender als NOK gilt
const unsigned long RF_TIMEOUT = 300;   // ms ohne Paket => RF: NOK

// Anzeige-Steuerung: aktuell angezeigter Sender & Haltezeit
uint8_t currentNode = 1;
unsigned long lastDisplaySwitch = 0;
const unsigned long DISPLAY_HOLD_TIME = 5000;  // 5 Sekunden pro Sender

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

void showStatusOnOLED(uint8_t id, bool rf_ok) {
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

void setup() {
  Serial.begin(115200);

  // --- OLED initialisieren ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // ggf. Adresse prüfen
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

  // lastSeen initialisieren
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
  lastDisplaySwitch = millis();
  showStatusOnOLED(currentNode, false);  // Start mit ID 1, RF: NOK
}

void loop() {
  unsigned long now = millis();

  // --- Pakete empfangen & lastSeen updaten ---
  while (radio.available()) {
    radio.read(&rx, sizeof(rx));

    // Nur IDs im erwarteten Bereich berücksichtigen
    if (rx.id >= 1 && rx.id <= NUM_NODES) {
      lastSeen[rx.id] = now;
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

  // --- Sender-Wechsel alle 5 Sekunden ---
  if (now - lastDisplaySwitch >= DISPLAY_HOLD_TIME) {
    currentNode++;
    if (currentNode > NUM_NODES) {
      currentNode = 1;
    }
    lastDisplaySwitch = now;
  }

  // --- RF-Status für aktuell angezeigten Sender bestimmen ---
  bool rf_ok = false;
  if (currentNode >= 1 && currentNode <= NUM_NODES) {
    if (now - lastSeen[currentNode] <= RF_TIMEOUT) {
      rf_ok = true;
    } else {
      rf_ok = false;
    }
  }

  // --- Anzeige aktualisieren ---
  // (kann man throttlen, aber 5 s Haltezeit macht es eh ruhig)
  static unsigned long lastDisplayUpdate = 0;
  if (now - lastDisplayUpdate > 200) {  // alle 200 ms Display neu zeichnen
    showStatusOnOLED(currentNode, rf_ok);
    lastDisplayUpdate = now;
  }

  delay(10);
}
