#include<Wire.h> //connects Pi to Arduino for LCD screen

#define SLAVE_ADDRESS 0x04
int data[32];
string angle;

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
    angle = Wire.read();
    Serial.print(“data received: “);
    Serial.println(angle);
    angleCHAR[i] = angle;
    i++; // need to reset i once we have the angle
   if (i==6){
   i=0;
   }
}
}
void sendData(){
  Wire.write(data[0]);
}
