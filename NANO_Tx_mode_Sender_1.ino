#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050_tockn.h>

RF24 radio(9, 10);                   // CE, CSN
const byte address[6] = "NOD01" ;   // Pipe-Adresse (gleich wie beim Empfänger)
MPU6050 mpu(Wire);
#define LED_GREEN 5
#define LED_RED 6

// Datenpaket: Rohdaten + Winkel (fixpoint: *100)
struct Payload {
  uint8_t id;  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t pitch, roll, yaw;  // in 0.01° (z.B. 1234 = 12.34°)
} payload;

void setup() {
  Serial.begin(115200);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // beide LEDs zunächst aus
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);

  Wire.begin();

  // MPU initialisieren & kurz kalibrieren
  digitalWrite(LED_RED, HIGH);          // 🔴 Rote LED AN -> Kalibrierung läuft
  mpu.begin();
  mpu.calcGyroOffsets(true);            // dauert einige Sekunden
  digitalWrite(LED_RED, LOW);           
  digitalWrite(LED_GREEN, HIGH);        
  delay(300);
  digitalWrite(LED_GREEN, LOW);

  // nRF24 initialisieren
  radio.begin();
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setPALevel(RF24_PA_LOW);        // bei Bedarf RF24_PA_HIGH
  radio.setDataRate(RF24_250KBPS);      // robust
  radio.setChannel(90);                 // kollisionsarm
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println(F("Nano-Sender 1 gestartet."));
}

void loop() {
  // Sensor updaten
  mpu.update();

  payload.id = 1;

  // Rohwerte (Gyro: °/s, Acc: raw)
  payload.ax = (int16_t)mpu.getAccX();  // tockn gibt i.d.R. bereits in 'g' skaliert aus;
  payload.ay = (int16_t)mpu.getAccY();  // für sehr große Dynamik ggf. skalieren/anpassen
  payload.az = (int16_t)mpu.getAccZ();

  payload.gx = (int16_t)mpu.getGyroX(); // °/s (Roh/skal.)
  payload.gy = (int16_t)mpu.getGyroY();
  payload.gz = (int16_t)mpu.getGyroZ();

  // Winkel als Fixpunkt (x100)
  payload.pitch = (int16_t)(mpu.getAngleX() * 100.0f);
  payload.roll  = (int16_t)(mpu.getAngleY() * 100.0f);
  payload.yaw   = (int16_t)(mpu.getAngleZ() * 100.0f);

  bool ok = radio.write(&payload, sizeof(payload));
  if (ok) {
  Serial.println(F("TX: OK (ACK)"));
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);
} else {
  Serial.println(F("TX: FEHLER"));
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, HIGH);
}

delay(100); // LED nur kurz an

delay(15);    // <<< HIER: Sende-Delay für Sender 1 (wichtig!)
}
