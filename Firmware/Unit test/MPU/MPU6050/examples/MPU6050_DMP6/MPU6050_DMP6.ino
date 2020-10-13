/*-----------------------------------------------------------------------------------------------------------------------------------------PREPROCESSER IMU*/

#define INTERRUPT_PIN 16

/*-----------------------------------------------------------------------------------------------------------------------------------------------------SETUP*/

void setup() {
  Serial.begin(115200);
  while (!Serial);

  initIMU();
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------------LOOP*/

void loop() {
  readIMU();


  Serial.print("Y ");
  Serial.print(getYaw());

  Serial.print("/  P ");
  Serial.print(getPitch());
  
  Serial.print("/  R ");
  Serial.println(getRoll());
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------------MORE FUNCTION*/
