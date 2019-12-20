// ADC Coverter 

#include <Wire.h>
#define SLAVE_ADRESS 0x08

int analogPin = A0;    // select the input pin for the potentiometer
int digitalPin_data = A4;   // pin for I2C comm SDA
int digitalPin_clk= A5      // pin for I2C comm SCL
byte sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  pinMode(analogPin,INPUT);
  pinMode(digitalPin_data,OUTPUT);
  pinMode(digitalPin_clk,OUTPUT);
  Wire.begin(SLAVE_ADRESS);                // join i2c bus with address #8
  Wire.onRequest(sendData); // register event
}

void sendData() {
  Wire.write(sensorValue); // respond with message of 6 bytes
  // as expected by master
}

void loop() {
  
byte  sensorValue = analogRead(analogPin);

}
