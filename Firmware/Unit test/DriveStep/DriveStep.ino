const int dirPin = 33; //8-33/25
const int stepPin = 23; //7-34->23/27

const int dirPin2 = 25; //8-33/25
const int stepPin2 = 27; //7-34/27

int i = 280;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

    pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
}
void loop() {
  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);

  for (int x = 0; x < 400; x++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(i);
    
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(i);
  }

  delay(1000);


  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, LOW);

  for (int x = 0; x < 400; x++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(i);
    
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(i);
  }
  delay(1000);


}
