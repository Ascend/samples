void setup() {
  Serial.begin(115200);         // start serial connection with baudrate 115200
}

void loop() {
  if(Serial.available() > 0){                         // check if there is bytes available to read in buffer
    String str = Serial.readString();                 // read bytes as string
    if(str.equals("Hello from Atlas 200 DK\n")){      // write a response if string received is "Hello from Atlas 200 DK"
      Serial.println("Arduino: Hello from Arduino");
    }
  }
}
