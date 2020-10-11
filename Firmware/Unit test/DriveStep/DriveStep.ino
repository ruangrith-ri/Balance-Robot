const int stepPin = 34; //7-34
const int dirPin = 33; //8-33
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);

  pinMode(35,INPUT);
Serial.begin(9600);
}
void loop() {
  int i = analogRead(35)/4.00;
 Serial.println(i);

  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(i); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(i); 
  }
  delay(2000); // One second delay
  
 /*
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 400; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(i);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(i);
  }
  delay(1000);
  
*/
 

}
