char auth[] = "V6tZhFZvi6ei4uk8FE8gQsw2UnZAo2gd";

void initBlynk() {
  Serial.println("Waiting for connections...");
  Blynk.setDeviceName("HCI-BalanceRobot1");
  Blynk.begin(auth);
}
