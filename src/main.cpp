#define m0 8
#define m1 9
#define aux 10

#include "Arduino.h"
#include "E220.h"

//running tx/rx on Serial 3
void readData() ;

void setup(){
    Serial.begin(9600);
    Serial3.begin(9600); //default baud is 9600
    pinMode(m0, OUTPUT);
    pinMode(m1, OUTPUT);
    pinMode(aux, INPUT);
    Serial.println("Started");

    Serial.println("Ready");
    Stream &mySerial = (Stream &)Serial3;
    E220 radioModule(&mySerial, m0, m1, aux);
    radioModule.init();
    Serial.println(radioModule.setAddress(25000, true));
    Serial.println(radioModule.getAddress());
    Serial.println(radioModule.setAirDataRate(ADR_9600,1));
    Serial.println(radioModule.getAirDataRate());
    Serial.println(radioModule.setPower(Power_21,1));
    Serial.println(radioModule.getPower(),BIN);
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
