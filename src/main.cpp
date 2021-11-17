#define m0_1 3
#define m1_1 2
#define aux_1 4

#define m0_3 8
#define m1_3 9
#define aux_3 10

#include "Arduino.h"
#include "E220.h"

//running tx/rx on Serial 3
void readData() ;

void setup(){
    Serial.begin(9600);
    Serial3.begin(9600); //default baud is 9600
    Serial1.begin(9600);


    Serial.println("Started");

    Serial.println("Ready");

    Stream &mySerial1 = (Stream &)Serial1;
    E220 radioModule1(&mySerial1, m0_1, m1_1, aux_1);

    Stream &mySerial3 = (Stream &)Serial3;
    E220 radioModule3(&mySerial3, m0_3, m1_3, aux_3);

    radioModule3.init();
    radioModule3.setAddress(3,1);
    Serial.println(radioModule3.getAddress());

    radioModule1.init();
    radioModule1.setAddress(1,1);
    Serial.println(radioModule1.getAddress());
    //radioModule3.

    /*
    digitalWrite(m0,HIGH);
    digitalWrite(m1,HIGH);
    delay(100);
    //WRITE
    byte message[3] = {0xC0, 0x00, 0x02};
    byte params [2] = {0x66, 0x99};
    Serial3.write(message, sizeof message);
    Serial3.write(params, sizeof params);
    readData();
    */
}
void loop(){

}

void readData(){
  delay(100);
  while(Serial3.available()){
    byte data = Serial3.read();
    Serial.print("Data: ");
    Serial.print(data, HEX);
    Serial.print("/");
    Serial.print(data, BIN);
    Serial.print("/");
    Serial.println(data, DEC);
  }
  Serial.println("DoneRead");
}
