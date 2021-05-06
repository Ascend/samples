#include <Wire.h>

void setup() {
   Wire.begin(4);
   Wire.onReceive(receiveData);
   Wire.onRequest(sendData);
   Serial.begin(115200);
}

void loop() {
  delay(100);
}

void receiveData(int numBytesReceived) {
  while(Wire.available()){
    char c = Wire.read();
    Serial.print(c); 
  }
}

void sendData() {
  Wire.write("Hello from Arduino\n");
}

