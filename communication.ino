#include<Wire.h> //connects Pi to Arduino for LCD screen

#define SLAVE_ADDRESS 0x04
int data[32];

void setup() {

  Wire.begin(SLAVE_ADDRESS); //sends data to LCD
  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);
  

}
void loop() {
delay(100);
}
void receiveData(int byteCount){

  while(Wire.available()) {
    number = Wire.read();
    Serial.print(“data received: “);
    Serial.println(number);

    if (number == 1){

      if (state == 0){
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      }
      else{
        digitalWrite(13, LOW); // set the LED off
          state = 0;
    }
      }
}
}
void sendData(){
  Wire.write(data[0]);
}
