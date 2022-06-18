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

    //Stream &mySerial = (Stream &)receiver;
    //E220 radioModule(&mySerial, m0, m1, aux);

    //initialise the module and check it communicates with us, else loop and keep trying
    while(!radioModule.init()){
        delay(5000);
    }
}


void loop(){
    //loop and look for user input on the serial port
    if(Serial.available()){
        //parse the serial message into a string and send it out to the module
        String message = Serial.readString();;
        mySerial.print(message);
        mySerial.flush();
        //output confirmation to the user that we have sent their message
        Serial.print("Sent: ");
        Serial.println(message);
    }
}