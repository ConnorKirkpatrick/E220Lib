//
// Created by connor on 15/11/2021.
//

#include "E220.h"

#include "Stream.h"
#include "UARTClass.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//create our new module object
E220::E220(Stream *s, int PIN_M0, int PIN_M1, int PIN_AUX){
    _streamSerial = s;
    _M0 = PIN_M0;
    _M1 = PIN_M1;
    _AUX = PIN_AUX;
    //_streamSerial->println("THIS IS A TEST");
}


//Initiate the module connected, basically see if we can read data
bool E220::init() {
    //set up the pins
    pinMode(_AUX, INPUT);
    pinMode(_M0, OUTPUT);
    pinMode(_M1, OUTPUT);

    setMode(MODE_NORMAL);
    bool check = readBoardData();
    if(!check){
        Serial.println("Issue initiating the module");
        return false;
    }
    else{
        return true;
    }

}



void E220::setMode(uint8_t mode){
    //time for the pins to recover, sheet says 2ms, 10 is safe
    switch (mode) {
        case MODE_NORMAL:
            digitalWrite(_M0, LOW);
            digitalWrite(_M1, LOW);
            break;
        case MODE_WOR_SENDING:
            digitalWrite(_M0, HIGH);
            digitalWrite(_M1, LOW);
            break;
        case MODE_WOR_RECEIVE:
            digitalWrite(_M0, LOW);
            digitalWrite(_M1, HIGH);
            break;
        case MODE_PROGRAM:
            digitalWrite(_M0, HIGH);
            digitalWrite(_M1, HIGH);
            break;
        default:
            digitalWrite(_M0, LOW);
            digitalWrite(_M1, LOW);
            break;
    }
    delay(20);
}

bool E220::readBoardData(){
    setMode(MODE_PROGRAM);
    byte configCommand[] = {0xC1, 0x00, 0x06};
    _streamSerial->write(configCommand, sizeof(configCommand));
    _streamSerial->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));
    //check the first 3 parts of the data are the same
    if((_Params[0] != 0xC1) | (_Params[1] != 0x00) | (_Params[2] != 0x06)){
        Serial.println("Error reading module config, check the wiring");
        return false;
    }
    else{
        //values in register 0&1
        _address =  (_Params[3] << 8) | (_Params[4]);
        //values in register 2
        //uart
        _baudRate = (_Params[5] & 0b11100000) >> 5;
        //Parity
        _parityBit = (_Params[5] & 0b00011000) >> 3;
        //ADR
        _airDataRate = (_Params[5] & 0b00000111);

        //values in register 3
        //Sub Packet
        _subPacketSize = (_Params[6] & 0b11000000) >> 6;
        //Ambient Noise
        _RSSIAmbientNoise = (_Params[6] & 0b00100000) >> 5;
        //Power Setting
        _transmitPower = (_Params[6] & 0b00000011);

        //values in register 4
        _channel = _Params[7];

        //values in register 5
        //RSSI Byte
        _RSSIByte = (_Params[8] & 0b10000000) >> 7;
        //Transmission Method
        _transmissionMethod = (_Params[8] & 0b01000000) >> 6;
        //LBT
        _LBTSetting = (_Params[8] & 0b00010000) >> 4;
        //WOR
        _WORCycle = (_Params[8] & 0b00000111);

        return true;
    }
}


