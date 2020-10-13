double sensed_output, control_signal, setpoint;
double Kp, Ki, Kd;

int T; //sample time in milliseconds (ms)
unsigned long last_time;
double totalError, lastError;
int max_control;
int min_control;

void setup() {

}

void loop() {
  PID_Control();
}

void PID_Control() {
  unsigned long current_time = millis();
  int delta_time = current_time - last_time;

  if (delta_time >= T) {

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
}
