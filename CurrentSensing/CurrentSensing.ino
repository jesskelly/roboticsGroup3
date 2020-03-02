/*CurrentSensing
  Reads the output of the current sensor in the analog input on pin 0,then
  converts it to voltage, and prints the result to the Serial port that goes
  in the raspberry.
*/

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  float voltage = sensorValue * (5.0 / 1023.0);

  Serial.println(voltage);
}
