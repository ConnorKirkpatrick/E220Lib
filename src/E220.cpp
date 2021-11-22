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
/**
 * Constructor for the E220 module
 * @param s The data stream, either a software of hardware serial
 * @param PIN_M0 The digital pin connected to pin M0 on the module
 * @param PIN_M1 The digital pin connected to pin M1 on the module
 * @param PIN_AUX The digital pin connected to pin AUX on the module
 */
E220::E220(Stream *s, int PIN_M0, int PIN_M1, int PIN_AUX){
    _streamSerial = s;
    _M0 = PIN_M0;
    _M1 = PIN_M1;
    _AUX = PIN_AUX;
}


//Initiate the module connected, basically see if we can read data
/**
 * Initializer for the module, used to check we can communicate with the module and read the current parameter
 * @return boolean representing the initialisation, true for success
 */
bool E220::init() {
    //set up the pins
    pinMode(_AUX, INPUT);
    pinMode(_M0, OUTPUT);
    pinMode(_M1, OUTPUT);
    //set the global board mode
    _setting = MODE_NORMAL;
    setMode(_setting);
    //read board default settings and assign global values
    bool check = readBoardData();
    //check if we were able to communicate with the board
    if(!check){
        Serial.println("Issue initiating the module");
        return false;
    }
    else{
        return true;
    }

}


/**
 * Set mode, public option to set the operating mode of the module
 * @param mode The mode to swap to
 */
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
    delay(50);
}

/**
 * Method reads all the default parameters from the module and updates the global values representing these
 * @return  boolean represening if the read was successful, true for success
 */
bool E220::readBoardData(){
    setMode(MODE_PROGRAM);
    byte configCommand[] = {0xC1, 0x00, 0x06};
    _streamSerial->write(configCommand, sizeof(configCommand));
    _streamSerial->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));
    //check the first 3 parts of the data are the same
    if((_Params[0] != 0xC1) | (_Params[1] != 0x00) | (_Params[2] != 0x06)){
        Serial.println("Error reading module config, check the wiring");
        setMode(_setting);
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

        setMode(_setting);
        return true;
    }
}
/**
 * Used to change the radio address of the module
 * @param newAddress new address in range 0-65535
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean represent the successful parameter change
 */
bool E220::setAddress(int newAddress, bool permanent) {
    if((newAddress > 65535) | (newAddress < 0)){
        Serial.println("Address out of range");
        return false;
    }
    uint16_t convAddress = newAddress;
    uint8_t addressH = ((convAddress & 0xFFFF) >> 8);
    uint8_t addressL = (convAddress & 0xFF);
    uint8_t addresses[] = {addressH, addressL};
    if(permanent){
        if(!writeCommand(0xC0, 0x00, 0x02, addresses)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x00, 0x02, addresses)){
            return false;
        }
    }
    // Write has succeeded, update the global parameters
    _address =  newAddress;
    return true;
}
/**
 * Getter for the parameter address
 * @return _address parameter
 */
uint16_t E220::getAddress() {
    return _address;
}

/**
 * private command for writing data to the module and verifying the response
 * @param cmdParam the command byte to use
 * @param address The address to start writing
 * @param length The length of parameter data to write
 * @param parameters The parameters in an array
 * @return boolean response for the success of the write command
 */
bool E220::writeCommand(uint8_t cmdParam, uint8_t address, uint8_t length, uint8_t *parameters) {
    setMode(MODE_PROGRAM);
    uint8_t message[3] = {cmdParam, address, length};
    _streamSerial->write(message, sizeof message);
    _streamSerial->write(parameters, length);

    //validate the output
    uint8_t output[sizeof(message)+length];
    _streamSerial->readBytes(output, sizeof(output));
    setMode(_setting);
    if((output[0] != 0xC1) or (output[1] != address) or (output[2] != length)){
        return false;
    }
    else{
        for(int i = 3; i < sizeof output; i++){
            if(output[i] != parameters[i-3]){
                return false;
            }
        }
        return true;
    }
}
/**
 * Used to set the Baud rate of the module Serial communication
 * @param newUART New baud rate to set
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setBaud(uint8_t newUART, bool permanent){
    uint8_t finalByte = newUART << 5;
    finalByte = finalByte | (_parityBit << 3);
    finalByte = finalByte | _airDataRate;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _baudRate = newUART;
    return true;
}
/**
 * Used to get the current Baud rate
 * @return The baud rate
 */
int E220::getBaud(){
    switch(_baudRate){
        case 0b000:
            return 1200;
        case 0b001:
            return 2400;
        case 0b010:
            return 4800;
        case 0b011:
            return 9600;
        case 0b100:
            return 19200;
        case 0b101:
            return 38400;
        case 0b110:
            return 57600;
        case 0b111:
            return 115200;
    }
    return _baudRate;
}
/**
 * Used to change the Serial parity bit of the module
 * @param newParity The new parity bit code
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setParity(uint8_t newParity, bool permanent) {
    uint8_t finalByte = _baudRate << 5;
    finalByte = finalByte | (newParity << 3);
    finalByte = finalByte | _airDataRate;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _parityBit = newParity;
    return true;
}
/**
 * Used to get the parity bit of the modules serial communication
 * @return The parity bit
 */
String E220::getParity() {
    switch(_parityBit){
        case 0b00:
            return "8N1";
        case 0b01:
            return "8O1";
        case 0b10:
            return "8E1";
        case 0b11:
            return "8N1"
    }
}
/**
 * Used to set the Air data rate (transmission rate) of the module
 * Higher rates mean lower ranges
 * @param newAirData The new air data rate
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setAirDataRate(uint8_t newAirData, bool permanent) {
    uint8_t finalByte = _baudRate << 5;
    finalByte = finalByte | (_parityBit << 3);
    finalByte = finalByte | newAirData;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x02, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _airDataRate = newAirData;
    return true;
}
/**
 * Used to get the Air data rate of the module
 * @return The air data rate in Hertz
 */
int E220::getAirDataRate() {
    switch(_airDataRate){
        case 0b000:
            return 300;
        case 0b001:
            return 1200;
        case 0b010:
            return 2400;
        case 0b011:
            return 4800;
        case 0b100:
            return 9600;
        case 0b101:
            return 19200;
        case 0b110:
            return 38400;
        case 0b111:
            return 62500;


    }
}

/**
 * Used to set the sub packet size of the module
 * @param newSize
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setSubPacketSize(uint8_t newSize, bool permanent) {
    uint8_t finalByte = newSize<< 6;
    finalByte = finalByte | (_RSSIAmbientNoise << 5);
    finalByte = finalByte | (0x1C);
    finalByte = finalByte | _transmitPower;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _subPacketSize = newSize;
    return true;
}
/**
 * Used to get the sub packet size of the module
 * @return The sub packet size
 */
int E220::getSubPacketSize() {
    switch(_subPacketSize){
        case 0b00:
            return 240;
        case 0b01:
            return 128;
        case 0b10:
            return 64;
        case 0b11:
            return 32;
    }
}

/**
 * Used to set the RSSI Ambient Noise registers of the module in normal mode
 * @param ambientSetting The desired setting of the Ambient noise setting
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setRSSIAmbient(uint8_t ambientSetting, bool permanent) {
    uint8_t finalByte = _subPacketSize<< 6;
    finalByte = finalByte | (ambientSetting << 5);
    finalByte = finalByte | (0x1C);
    finalByte = finalByte | _transmitPower;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _RSSIAmbientNoise = ambientSetting;
    return true;
}
/**
 * Used to get the RSSI Ambient noise setting of the module
 * @return The Ambient Noise setting
 */
uint8_t E220::getRSSIAmbient() {
    return _RSSIAmbientNoise;
}

/**
 * Used to set the broadcast power of the module
 * Higher power equates to greater range but greater power draw
 * @param ambientSetting The desired power setting
 * @param permanent Set this as a non-volatile parameter
 * @return Boolean success parameter
 */
bool E220::setPower(uint8_t newPower, bool permanent) {
    uint8_t finalByte = _subPacketSize<< 6;
    finalByte = finalByte | (_RSSIAmbientNoise << 5);
    finalByte = finalByte | (0x1C);
    finalByte = finalByte | newPower;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x03, 0x01, registerParams)){
            return false;
        }
    }
    //success, update the global params
    _transmitPower = newPower;
    return true;
}

/**
 * Used to get the transmission power setting of the module
 * @return The Transmission power setting in dBm
 */
int E220::getPower() {
    switch(_transmitPower){
        case 0b00:
            return 30;
        case 0b01:
            return 27;
        case 0b10:
            return 24;
        case 0b11:
            return 21;
    }
}

void E220::printBoardParmeters() {
    setMode(MODE_PROGRAM);
    byte configCommand[] = {0xC1, 0x00, 0x06};
    _streamSerial->write(configCommand, sizeof(configCommand));
    _streamSerial->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));
    //check the first 3 parts of the data are the same
    if((_Params[0] != 0xC1) | (_Params[1] != 0x00) | (_Params[2] != 0x06)){
        Serial.println("Error reading module config, check the wiring");
        setMode(_setting);
    }
    else{
        //print the raw output
        Serial.println("Raw Data:");
        Serial.println("HEX/BINARY/DECIMAL");
        for(int i = 0; i < sizeof _Params; i++){
            Serial.print(_Params[i], HEX);
            Serial.print("/");
            Serial.print(_Params[i], BIN);
            Serial.print("/");
            Serial.println(_Params[i], DEC);
        }
        Serial.println("");
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

        Serial.print("Address: ");
        Serial.println(_address);
        Serial.print("Baud Rate setting: ");
        Serial.println(_baudRate, BIN);
        Serial.print("Parity Bit setting: ");
        Serial.println(_parityBit, BIN);
        Serial.print("Air Data Rate setting: ");
        Serial.println(_airDataRate, BIN);
        Serial.print("Sub Packet Size setting: ");
        Serial.println(_subPacketSize, BIN);
        Serial.print("RSSI Ambient Noise Toggle: ");
        Serial.println(_RSSIAmbientNoise, BIN);
        Serial.print("Transmission Power: ");
        Serial.println(_transmitPower, BIN);
        Serial.print("Channel: ");
        Serial.println(_channel);
        Serial.print("RSSI Byte Toggle: ");
        Serial.println(_RSSIByte, BIN);
        Serial.print("Transmission Mode Toggle (Fixed = 1): ");
        Serial.println(_transmissionMethod, BIN);
        Serial.print("LBT Monitoring Toggle: ");
        Serial.println(_LBTSetting, BIN);
        Serial.print("WOR Cycle setting: ");
        Serial.println(_WORCycle, HEX);
    }
}


//TODO method to print raw data out of the registers for verification
///Maybe change the returns to use a switch to return a textual version of the data rather than the raw binary?
///EG switch on the air data, return the baud rate as 9600 rather than 010


