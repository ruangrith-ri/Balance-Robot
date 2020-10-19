#define BLYNK_PRINT Serial
#define BLYNK_USE_DIRECT_CONNECT

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define INTERRUPT_PIN 16
#define CHANNEL 0
#define BITM  13
#define FREQ 5000
#define RD 27
#define RP 14
#define LD 12
#define LP 13

double sensed_output, control_signal, setpoint;
double Kp = 0, Ki = 0, Kd = 0;

int T = 0; //sample time in milliseconds (ms)
unsigned long last_time;
double totalError, lastError;
int max_control = 8000;
int min_control = -8000;


/*-----------------------------------------------------------------------------------------------------------------------------------------------------TASK*/

TaskHandle_t Task1;

void Task1code( void * parameter) {

  Serial.print("Compute Core : ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    readIMU();

    sensed_output = getPitch();
    PID_Control();


    if (sensed_output >= -60 && sensed_output <= 60) {
      if (control_signal > 0 ) {
        Direction("Forward", control_signal, control_signal);
        //Serial.println("A");
      } else if (control_signal < 0) {
        Direction("Backward", -control_signal, -control_signal);
        //Serial.println("B");
      }
    } else {
      Direction("Forward", 0, 0);
      //Serial.println("C");

    }
  }
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------------SETUP*/

void setup() {
  Serial.begin(9600);

  SetMotor();

  initIMU();

  initBlynk();
  delay(500);

  Serial.print("Main Core : ");
  Serial.println(xPortGetCoreID());

  xTaskCreatePinnedToCore(
    Task1code, /* Function to implement the task */
    "Task1", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    10,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */
}

/*-----------------------------------------------------------------------------------------------------------------------------------------------------LOOP*/

int delta_time_monitor = 0 , core0_time_monitor = 0;

void loop() {
  Blynk.run();

  Blynk.virtualWrite(V0, control_signal);
  Blynk.virtualWrite(V1, sensed_output);

  Blynk.virtualWrite(V2, delta_time_monitor);
  Blynk.virtualWrite(V4, core0_time_monitor);
}

BLYNK_WRITE(V11) {
  Kp = param.asDouble();
  Serial.print("Blynk Core : ");
  Serial.println(xPortGetCoreID());
}

BLYNK_WRITE(V12) {
  Ki = param.asDouble();
}

BLYNK_WRITE(V13) {
  Kd = param.asDouble();
}

BLYNK_WRITE(V14) {
  T = param.asInt();
}





long core_last_time = 0;

void PID_Control() {
  unsigned long current_time = millis();
  int delta_time = current_time - last_time;

  core0_time_monitor = current_time - core_last_time ;

  if (delta_time >= T) {

    delta_time_monitor = delta_time;

    double error = setpoint - sensed_output;

    totalError += error;

    if (totalError >= max_control) totalError = max_control;
    else if (totalError <= min_control) totalError = min_control;

    double deltaError = error - lastError;

    control_signal = Kp * error + (Ki * T) * totalError + (Kd / T) * deltaError;

    if (control_signal >= max_control) {
      control_signal = max_control;
    } else if (control_signal <= min_control) {
      control_signal = min_control;
    }

    lastError = error;
    last_time = current_time;
  }
  core_last_time = current_time;
}







void AnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  ledcWrite(channel, duty);
}

void SetMotor() {
  ledcSetup(CHANNEL, FREQ, BITM);
  ledcAttachPin(RP, CHANNEL);
  pinMode(RD, OUTPUT);
  ledcAttachPin(LP, CHANNEL);
  pinMode(LD, OUTPUT);
}

void Direction(String Mode, int PWML, int PWMR) {
  if (Mode == "Forward") {
    digitalWrite(RD, LOW);
    AnalogWrite(CHANNEL, PWMR);
    digitalWrite(LD, HIGH);
    AnalogWrite(CHANNEL, PWML);
  } else if (Mode == "Backward") {
    digitalWrite(RD, HIGH);
    AnalogWrite(CHANNEL, PWMR);
    digitalWrite(LD, LOW);
    AnalogWrite(CHANNEL, PWML);
  } else if (Mode == "Rotateleft") {
    digitalWrite(RD, LOW);
    AnalogWrite(CHANNEL, PWMR);
    digitalWrite(LD, LOW);
    AnalogWrite(CHANNEL, PWML);
  } else if (Mode == "Rotateright") {
    digitalWrite(RD, HIGH);
    AnalogWrite(CHANNEL, PWMR);
    digitalWrite(LD, HIGH);
    AnalogWrite(CHANNEL, PWML);
  }
}
