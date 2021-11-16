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
      //set system mode 3 to configure
      digitalWrite(m0,HIGH);
      digitalWrite(m1,HIGH);
      delay(1000);
      byte message[] = {0xC1, 0x00, 0xFF, 0xFF};
      Serial3.write(message, sizeof(message));
      readData();

/*  Serial.println("New Message");
  byte newMessage[] = {0xC1, 0X00, 0x06};
  Serial3.write(newMessage, sizeof(newMessage));
  Serial.println("Message Sent");*/

/*  byte param[9];
  Serial3.readBytes((uint8_t*)&param, (uint8_t)sizeof param);
  for (int i = 0; i < 9; i++){
      Serial.print(param[i], HEX);
      Serial.print("/");
      Serial.print(param[i], BIN);
      Serial.print("/");
      Serial.println(param[i], DEC);
  }
  Serial.println("\n\n\n");
  //readData();*/
    Serial.println("Ready");
    //Stream &mySerial = (Stream &)Serial;
    Stream &mySerial = (Stream &)Serial3;
    E220 radioModule(&mySerial, m0, m1, aux);
    radioModule.init();
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
