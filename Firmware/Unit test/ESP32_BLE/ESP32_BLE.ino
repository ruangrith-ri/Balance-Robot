#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

char auth[] = "V6tZhFZvi6ei4uk8FE8gQsw2UnZAo2gd";

void setup(){
  Serial.begin(9600);
  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("HCI-BalanceRobot1");
  Blynk.begin(auth);
}

int i = 0;
void loop(){
  Blynk.run();
  
  Blynk.virtualWrite(V1, i++);
    //Serial.println(i);
}
