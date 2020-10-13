#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define INTERRUPT_PIN 16

void setup() {
  //Serial.begin(115200);
  initIMU();
  initBlynk();
}

void loop() {
  Blynk.run();
  readIMU();

  Blynk.virtualWrite(V0, getYaw());
  Blynk.virtualWrite(V1, getPitch());
  Blynk.virtualWrite(V2, getRoll());
}
