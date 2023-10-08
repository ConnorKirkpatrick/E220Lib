/**
 *  Simple library used for the configuration of EBYTE E220 modules
 *  @author Connor Kirkpatrick
 *  @date 15/11/2021
 */
#include "E220.h"

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
    while(digitalRead(_AUX) == LOW){
      delayMicroseconds(1); //if the AUX pin is low this means some data is still being written, don't change the module settings
    }
    //time for the pins to recover, sheet says 2ms, 10 is safe
    delay(20);
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
    while(digitalRead(_AUX) == LOW){
      delay(1); //if the AUX pin is low this means the module is still processing the change
    }
    delay(2); //delay as stated on data sheet, aux high must last for 2ms before mode change is complete
}
/**
 * setRadioMode, public option to set the operating mode of the module
 * @param mode The mode to swap to
 */
bool E220::setRadioMode(uint8_t mode){
  _setting = mode;
  setMode(_setting);
  return true;
}
/**
 * Method reads all the default parameters from the module and updates the global values representing these
 * @return  boolean representing if the read was successful, true for success
 */
bool E220::readBoardData(){
    setMode(MODE_PROGRAM);
    byte configCommand[] = {0xC1, 0x00, 0x06};
    _streamSerial->write(configCommand, sizeof(configCommand));
    _streamSerial->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));
    _streamSerial->flush();
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
 * private command for writing data to the module and verifying the response
 * @param cmdParam the command byte to use
 * @param address The address to start writing
 * @param length The length of parameter data to write
 * @param parameters The parameters in an array
 * @return boolean response for the success of the write command
 */
bool E220::writeCommand(uint8_t cmdParam, uint8_t address, uint8_t length, uint8_t *parameters) {
    setMode(MODE_PROGRAM);
    while(_streamSerial->available()){
      _streamSerial->read(); //do this to clear the serial and thus prevent reading of incorrect data
    }
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
 * Used to change the radio address of the module
 * @param newAddress {int} new address in range 0-65535
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return Boolean represent the successful parameter change
 */
bool E220::setAddress(unsigned int newAddress, bool permanent) {
    if((newAddress > 65535)){
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
 * Used to set the Baud rate of the module Serial communication
 * @param newUART {UDR_*} New baud rate to set
 * @param permanent {bool} Set this as a non-volatile parameter
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
 * @param newParity {PB_*} The new parity bit code
 * @param permanent {bool} Set this as a non-volatile parameter
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
            return "8N1";
    }
}
/**
 * Used to set the Air data rate (transmission rate) of the module
 * Higher rates mean lower ranges
 * @param newAirData {ADR_*} The new air data rate
 * @param permanent {bool} Set this as a non-volatile parameter
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
 * @param newSize {SPS_*} The new size of the subpackets when divided
 * @param permanent {bool} Set this as a non-volatile parameter
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
 * @param ambientSetting {RAN_*} The desired setting of the Ambient noise setting
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
 * Function used to read the ambient and latest RSSI from the module
 * The module must have RSSIAmbient enabled and be in either Normal or WOR SENDING mode
 * Note: The module will take a reading for 1 second to determine ambient RSSI
 * @return {uint16_t} 2 bytes of data, high byte is the environmental noise, low byte is the most recent message RSSI
 */
 //write: 0
//read:
uint16_t E220::readRSSIAmbient() {
  while(digitalRead(_AUX) == LOW){
    delayMicroseconds(1);
  }
  if(!getRSSIAmbient()){
    Serial.println("RSSI Ambient not enabled");
    return 0xFFFF;
  }
  if(_setting != MODE_NORMAL & _setting != MODE_WOR_SENDING){
    Serial.println("Module not in the correct mode, must be in normal or WOR sending");
    return 0xFFFF;
  }
  uint8_t response[5];
  uint8_t data[6] = {0xC0,0xC1,0xC2,0xC3,0x00,0x02};
  _streamSerial->write(data,6);
  _streamSerial->readBytes(response,6);
  if(response[0] != 0xC1 | response[1] != 0x00 | response[2] != 0x02){
    return 0xFFFF;
  }
  return response[3] <<8 | response[4];
}

/**
 * Used to set the broadcast power of the module
 * Higher power equates to greater range but greater power draw
 * @param ambientSetting {Power_*} The desired power setting
 * @param permanent {bool} Set this as a non-volatile parameter
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

/**
 * Function used to set the channel between 0-80 in register 4
 * @param newChannel {int} the new channel
 * @param permanent {bool} this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setChannel(int newChannel, bool permanent) {

    if(newChannel < 0 | newChannel > 83){
        Serial.print("Channel out of range 0-83");
        return false;
    }
    else{
        uint8_t registerParams[] = {static_cast<uint8_t>(newChannel)};
        if(permanent){
            if(!writeCommand(0xC0, 0x04, 0x01, registerParams)){
                return false;
            }
        }
        else{
            if(!writeCommand(0xC2, 0x04, 0x01, registerParams)){
                return false;
            }
        }
        _channel = newChannel;
        return true;
    }
}

/**
 * getter for the channel parameter
 * @return {int} The channel
 */
int E220::getChannel() {
    return _channel;
}

/**
 * Setter for the RSSIByte toggle
 * @param setting {bool} The desired setting
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setRSSIByteToggle(bool Setting, bool permanent) {
    uint8_t toggle = 0b0;
    if(Setting){toggle = 0b1;}
    uint8_t finalByte = toggle << 7;
    finalByte = finalByte | (_transmissionMethod << 6);
    finalByte = finalByte | (_LBTSetting << 4);
    finalByte = finalByte | _WORCycle;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    _RSSIByte = Setting;
    return true;
}

/**
 * getter for the RSSIByte toggle
 * @return {int} The RSSIByte toggle
 */
bool E220::getRSSIByteToggle() {
    return _RSSIByte;
}

/**
 * Setter for the TransmissionMode toggle
 * @param setting {bool} The desired setting
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setFixedTransmission(bool Setting, bool permanent) {
    uint8_t toggle = 0b0;
    if(Setting){toggle = 0b1;}

    uint8_t finalByte = _RSSIByte << 7;
    finalByte = finalByte | (toggle << 6);
    finalByte = finalByte | (_LBTSetting << 4);
    finalByte = finalByte | _WORCycle;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    _transmissionMethod = Setting;
    return true;

}
/**
 * getter for the Transmission Method
 * @return {int} The transmission method
 */
bool E220::getFixedTransmission() {
    return _transmissionMethod;
}

/**
 * Setter for the LBT toggle
 * @param setting {bool} The desired setting
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setLBT(bool Setting, bool permanent) {
    uint8_t toggle = 0b0;
    if(Setting){toggle = 0b1;}

    uint8_t finalByte = _RSSIByte << 7;
    finalByte = finalByte | (_transmissionMethod << 6);
    finalByte = finalByte | (toggle << 4);
    finalByte = finalByte | _WORCycle;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    _LBTSetting = Setting;
    return true;
}

/**
 * getter for the LBT Setting
 * @return {int} The LBT Setting
 */
bool E220::getLBT() {
    return _LBTSetting;
}

/**
 * Setter for the WOR Cycle setting
 * @param Setting {bool} The desired setting
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setWORCycle(uint8_t WORSetting, bool permanent) {
    uint8_t finalByte = _RSSIByte << 7;
    finalByte = finalByte | (_transmissionMethod << 6);
    finalByte = finalByte | (_LBTSetting << 4);
    finalByte = finalByte | WORSetting;
    uint8_t registerParams[] = {finalByte};
    if(permanent){
        if(!writeCommand(0xC0, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x05, 0x01, registerParams)){
            return false;
        }
    }
    _WORCycle = WORSetting;
    return true;
}
/**
 * getter for the Wor Cycle
 * @return {int} The WOR Cycle
 */
int E220::getWORCycle() {
    switch(_WORCycle){
        case 0b000:
            return 500;
        case 0b001:
            return 1000;
        case 0b010:
            return 1500;
        case 0b011:
            return 2000;
        case 0b100:
            return 2500;
        case 0b101:
            return 3000;
        case 0b110:
            return 3500;
        case 0b111:
            return 4000;

    }
}
/**
 * Method used to set the encryption key for transmission
 * @param key {unsigned char} The encryption key
 * @param permanent {bool} Set this as a non-volatile parameter
 * @return {bool} the success factor
 */
bool E220::setEncryptionKey(unsigned int key, bool permanent) {
    //convert key to hex
    uint8_t keyH = highByte(key);
    uint8_t keyL = lowByte(key);
    uint8_t keys[] = {keyH, keyL};
    if(permanent){
        if(!writeCommand(0xC0, 0x06, 0x02, keys)){
            return false;
        }
    }
    else{
        if(!writeCommand(0xC2, 0x06, 0x02, keys)){
            return false;
        }
    }
    return true;
}

/**
 * Method used to print the raw parameters from the module, mainly used for sanity checking
 */
void E220::printBoardParameters() {
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
        //print the raw output for debug
        /*
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
         */
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
        switch (_baudRate) {
            case 0b000:
                Serial.println("UDR_1200");
                break;
            case 0b001:
                Serial.println("UDR_2400");
                break;
            case 0b010:
                Serial.println("UDR_4800");
                break;
            case 0b011:
                Serial.println("UDR_9600");
                break;
            case 0b100:
                Serial.println("UDR_19200");
                break;
            case 0b101:
                Serial.println("UDR_38400");
                break;
            case 0b110:
                Serial.println("UDR_57600");
                break;
            case 0b111:
                Serial.println("UDR_115200");
                break;
        }
        Serial.print("Parity Bit setting: ");
        switch (_parityBit) {
            case 0b00:
                Serial.println("PB_8N1");
                break;
            case 0b01:
                Serial.println("PB_8O1");
                break;
            case 0b11:
                Serial.println("PB_8E1");
                break;
        }
        Serial.print("Air Data Rate setting: ");
        switch (_airDataRate) {
            case 0b010:
                Serial.println("ADR_2400");
                break;
            case 0b011:
                Serial.println("ADR_4800");
                break;
            case 0b100:
                Serial.println("ADR_9600");
                break;
            case 0b101:
                Serial.println("ADR_19200");
                break;
            case 0b110:
                Serial.println("ADR_38400");
                break;
            case 0b111:
                Serial.println("ADR_62500");
                break;
        }
        Serial.print("Sub Packet Size setting: ");
        switch (_subPacketSize) {
            case 0b00:
                Serial.println("SPS_200");
                break;
            case 0b01:
                Serial.println("SPS_128");
                break;
            case 0b10:
                Serial.println("SPS_64");
                break;
            case 0b11:
                Serial.println("SPS_32");
                break;
        }
        Serial.print("RSSI Ambient Noise Toggle: ");
        switch (_subPacketSize) {
            case 0b00:
                Serial.println("Disabled");
                break;
            case 0b01:
                Serial.println("Enabled");
                break;
        }
        Serial.print("Transmission Power: ");
        switch (_transmitPower) {
            case 0b00:
                Serial.println("Power_30");
                break;
            case 0b01:
                Serial.println("Power_27");
                break;
            case 0b10:
                Serial.println("Power_24");
                break;
            case 0b11:
                Serial.println("Power_21");
                break;
        }
        Serial.print("Channel: ");
        Serial.println(_channel);
        Serial.print("RSSI Byte Toggle: ");
        switch (_RSSIByte) {
            case 0b00:
                Serial.println("Disabled");
                break;
            case 0b01:
                Serial.println("Enabled");
                break;
        }
        Serial.print("Transmission Mode Toggle: ");
        switch (_transmissionMethod) {
            case 0b00:
                Serial.println("Disabled");
                break;
            case 0b01:
                Serial.println("Enabled");
                break;
        }
        Serial.print("LBT Monitoring Toggle: ");
        switch (_LBTSetting) {
            case 0b00:
                Serial.println("Disabled");
                break;
            case 0b01:
                Serial.println("Enabled");
                break;
        }
        Serial.print("WOR Cycle setting: ");
        switch (_WORCycle) {
            case 0b000:
                Serial.println("WOR500");
                break;
            case 0b001:
                Serial.println("WOR1000");
                break;
            case 0b010:
                Serial.println("WOR1500");
                break;
            case 0b011:
                Serial.println("WOR2000");
                break;
            case 0b100:
                Serial.println("WOR2500");
                break;
            case 0b101:
                Serial.println("WOR3000");
                break;
            case 0b110:
                Serial.println("WOR3500");
                break;
            case 0b111:
                Serial.println("WOR4000");
                break;
        }
        setMode(_setting);
    }
}
/**
 * Used to set the escape character that is used to denote the end of a message
 * @param character {char} the character to set
 * @return {bool} true upon success
 */
bool E220::setEscapeCharacter(char character) {
  escapeCharacter = character;
  return true;
}
/**
 * getter used to check the currently set escape character
 * @return {uint8_t} the escape character
 */
uint8_t E220::getEscapeCharacter() {
  return escapeCharacter;
}

/**
 * Function used to send string data in a transparent way. The module must
 * either not be in fixed transmission mode or have a broadcast address of
 * 0xFFFF. The set escape character will be automatically added.
 * @param data {String} The string object to send
 * @return {bool} returns false if an error occurs
 */
bool E220::sendTransparentData(String data) {
  if(!getFixedTransmission() || getAddress() == 0xFFFF) {
    _streamSerial->print(data+escapeCharacter);
    _streamSerial->flush();
    return true;
  }
  else{
    Serial.println("Not in transparent mode, set fixedTransmission to false or address to 0xFFFF");
    return false;
  }
}

/**
 * Function used to send a byte array in a transparent way. The module must
 * either not be in fixed transmission mode or have a broadcast address of
 * 0xFFFF. The set escape character will be automatically added.
 * @param data {uint8_t *}The byte array to send
 * @param size {int} The size of the array
 * @return {bool} returns false if an error occurs
 */
bool E220::sendTransparentData(uint8_t *data, int size) {
  if(!getFixedTransmission() || getAddress() == 0xFFFF) {
    for(int i = 0; i < size; i++){
      _streamSerial->write(data[i]);
    }
    _streamSerial->write(escapeCharacter);
    _streamSerial->flush();
    return true;
  }
  else{
    Serial.println("Not in transparent mode, set fixedTransmission to false or address to 0xFFFF");
    return false;
  }
}

/**
 * Function used to send data to a fixed address or channel. Requires the module
 * to be in fixed transmission mode. The device needs to be given an address and
 * channel to broadcast to. The first 3 bytes of the data sent to the module
 * represent the address and channel. The auxAvailable flag is used to show if
 * the aux port is connected, and thus if it can be used for timing control.
 * @param address {int} The address of the targeted device. Can be broadcast
 * address (OxFFFF)
 * @param channel {int} The channel for the targeted transmission.
 * @param data  {String} The string of data to send
 * @param auxAvailable {bool} Flag used to enable of disable auxiliary timing
 * control
 * @return {bool} returns false if an error occurs
 */
bool E220::sendFixedData(unsigned int address, int channel, String data, bool auxAvailable) {
  if(address > 65535){
    Serial.println("Address out of range");
    return false;
  }
  if(channel > 80 | channel < 0){
    Serial.println("Channel out of range");
    return false;
  }
  if(!getFixedTransmission()){
    Serial.println("Not in fixed transmission mode");
    return false;
  }
  if(auxAvailable) {
    // pause if data is still being sent to ensure this is read as a new message
    while (digitalRead(_AUX) == LOW) {
      delay(10);
    }
  }
  uint8_t message[data.length()+4];
  message[0] = highByte(address);
  message[1] = lowByte(address);
  message[2] = (uint8_t) channel;
  memcpy(&message[3],data.c_str(),data.length());
  message[data.length()+3] = escapeCharacter;
  _streamSerial->write(message,data.length()+4);
  _streamSerial->flush();
  return true;
}

/**
 * Function used to send data to a fixed address or channel. Requires the module
 * to be in fixed transmission mode. The device needs to be given an address and
 * channel to broadcast to. The first 3 bytes of the data sent to the module
 * represent the address and channel. The auxAvailable flag is used to show if
 * the aux port is connected, and thus if it can be used for timing control.
 * @param address {int} The address of the targeted device. Can be broadcast
 * address (OxFFFF)
 * @param channel {int} The channel for the targeted transmission.
 * @param data {uint8_t*} The array of data to send
 * @param size {int} The size of the array of data
 * @param auxAvailable {bool} Flag used to enable of disable auxiliary timing
 * control
 * @return {bool} returns false if an error occurs
 */
bool E220::sendFixedData(unsigned int address, int channel, uint8_t *data, int size, bool auxAvailable) {
  if(address > 65535){
    Serial.println("Address out of range");
    return false;
  }
  if(channel > 80 | channel < 0){
    Serial.println("Channel out of range");
    return false;
  }
  if(!getFixedTransmission()){
    Serial.println("Not in fixed transmission mode");
    return false;
  }
  if(auxAvailable) {
    // pause if data is still being sent to ensure this is read as a new message
    while (digitalRead(_AUX) == LOW) {
      delay(10);
    }
  }
  uint8_t message[size+4];
  message[0] = highByte(address);
  message[1] = lowByte(address);
  message[2] = (uint8_t) channel;
  memcpy(&message[3],data,size);
  message[size+3] = escapeCharacter;
  _streamSerial->write(message,size+4);
  _streamSerial->flush();
  return true;
}

/**
 * Function used to read a string of data from the radio module upto the first
 * instance of the escape character.
 * @return {String} The data from the module.
 */
String E220::receiveData() {
  if(_streamSerial->available()){
    return _streamSerial->readStringUntil(escapeCharacter);
  }else{
    return "";
  }
}
/**
 * Function used to read a set of data from the radio module upto the first
 * instance of the escape character.
 * @param data {uint_8*} The array to write the data into
 * @param size {int} The size of the array
 * @return {bool} A flag representing whether data was read or not
 */
bool E220::receiveData(uint8_t *data, int size) {
  if(_streamSerial->available()){
   _streamSerial->readBytes(data,size);
   return true;
  }
  else{
    return false;
  }
}





