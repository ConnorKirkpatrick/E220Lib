#include <Arduino.h>
#include "E220.h"
// #include <SoftwareSerial.h>

//Define the pins we need to use later to create the object
#define m0 7
#define m1 6
#define aux 5

//SoftwareSerial receiver(3,2)

void setup(){
    //begin all of our UART connections
    Serial.begin(9600);
    Serial3.begin(9600); //default baud is 9600
    //receiver.begin(9600);

    //initiate the radio module
    Stream &mySerial = (Stream &)Serial3;
    E220 radioModule(&mySerial, m0, m1, aux);
    //E220 radioModule(receiver, m0, m1, aux);

    //initialise the module and check it communicates with us, else loop and keep trying
    while(!radioModule.init()){
        delay(5000);
    }
}


void loop(){
    //loop and keep checking for new messages
    if(Serial3.available()){
        Serial.print("Messaged Received: ");
        Serial.println(Serial3.readString());
    }
}