/**
 *  Simple library used for the configuration of EBYTE E220 modules
 *  @author Connor Kirkpatrick
 *  @date 15/11/2021
 */

//Define all my constants

#include "USB/USBAPI.h"

#define MODE_NORMAL 0			// can send and receive data
#define MODE_WOR_SENDING 1	    // sends a preamble to waken receiver
#define MODE_WOR_RECEIVE 2		// can't transmit, Can only receive from transmitter in mode 1. System only checks for incoming every WOR cycle
#define MODE_PROGRAM 3          // Power saving mode, also used to change the parameters of the device

//Registers 0 and 1 reserved for the module address
//65536 possible addresses

//Register 2
//UART data rates
// (can be different for transmitter and receiver)
#define UDR_1200 0b000		// 1200 baud
#define UDR_2400 0b001		// 2400 baud
#define UDR_4800 0b010		// 4800 baud
#define UDR_9600 0b011		// 9600 baud default
#define UDR_19200 0b100		// 19200 baud
#define UDR_38400 0b101		// 34800 baud
#define UDR_57600 0b110		// 57600 baud
#define UDR_115200 0b111	// 115200 baud

// parity bit options (must be the same for transmitter and receiver)
#define PB_8N1 0b00			// default
#define PB_8O1 0b01
#define PB_8E1 0b11

// air data rates (certain types of modules)
// (must be the same for transmitter and receiver)
#define ADR_2400 0b010		    // 2400 baud
#define ADR_4800 0b011		    // 4800 baud
#define ADR_9600 0b100		    // 9600 baud
#define ADR_19200 0b101		    // 19200 baud
#define ADR_38400 0b110		    // 38400 baud
#define ADR_62500 0b111	        // 62500 baud

//Register 3
//sub-packet size setting
#define SPS_200 0b00        //200 bytes
#define SPS_128 0b01        //128bytes
#define SPS_64 0b10         //64bytes
#define SPS_32 0b11         //32bytes

//RSSI Ambient Noise Enable
#define RAN_D 0b00          //disabled (default)
#define RAN_E 0b01          //Enabled

//Transmission Power
#define Power_30 0b00       //30dBm (Default)
#define Power_27 0b01       //27dBm
#define Power_24 0b10       //24dBm
#define Power_21 0b11       //21dBm

//Register 4 reserved for the channel
//16 bits of data for the channel

//Register 5
//RSSI Byte
#define RSSIB_D 0b00        //Disabled (default)
#define RSSIB_E 0b01        //Enabled; after any message is received, module transmits data on TX followed by the RSSI strength byte

//Transmission Mode
#define Transparent 0b00    //Transparent transmission (default)
#define Fixed   0b01        //Fixed transmission; Module will take the first 3 bytes on serial as address high+low+channel

//LBT
#define LBT_D 0b00          //No LBT Monitoring
#define LBT_E 0b01          //LBT Enabled, system will monitor the channel to try to avoid interference when transmitting, can cause delays of upto 2 seconds max

//WOR Cycle
#define WOR500 0b000        //WOR cycle awake every 500ms
#define WOR1000 0b001       //WOR cycle awake every 1000ms
#define WOR1500 0b010       //WOR cycle awake every 1500ms
#define WOR2000 0b011       //WOR cycle awake every 2000ms
#define WOR2500 0b100       //WOR cycle awake every 2500ms
#define WOR3000 0b101       //WOR cycle awake every 3000ms
#define WOR3500 0b110       //WOR cycle awake every 3500ms
#define WOR4000 0b111       //WOR cycle awake every 4000ms

//Register 6
//cryptoKey high and low, the two halves of the crypto key
//16 bits for the total key


class E220 {
    private:

    int _M0;
    int _M1;
    int _AUX;

    Serial_ *_streamSerial;
    uint8_t _Params[9];
    uint8_t _setting;

    uint16_t _address;
    uint8_t _baudRate;
    uint8_t _parityBit;
    uint8_t _airDataRate;
    uint8_t _subPacketSize;
    uint8_t _RSSIAmbientNoise;
    uint8_t _transmitPower;
    uint8_t _channel;
    uint8_t _RSSIByte;
    uint8_t _transmissionMethod;
    uint8_t _LBTSetting;
    uint8_t _WORCycle;

    char escapeCharacter = '\n';

    bool writeCommand(uint8_t cmdParam, uint8_t address, uint8_t length, uint8_t parameters[]);

    public:
        E220(Serial_ *s, int PIN_M0, int PIN_M1, int PIN_AUX);

        bool init();
        void setMode(uint8_t mode);
        bool readBoardData();

        bool setAddress(int newAddress, bool permanent);
        uint16_t getAddress();

        bool setBaud(uint8_t newUART, bool permanent);
        int getBaud();

        bool setParity(uint8_t newParity, bool permanent);
        String getParity();

        bool setAirDataRate(uint8_t newAirData, bool permanent);
        int getAirDataRate();

        bool setSubPacketSize(uint8_t newSize, bool permanent);
        int getSubPacketSize();

        bool setRSSIAmbient(uint8_t ambientSetting, bool permanent);
        uint8_t getRSSIAmbient();

        bool setPower(uint8_t newPower, bool permanent);
        int getPower();

        bool setChannel(int newChannel, bool permanent);
        int getChannel();

        bool setRSSIByteToggle(bool Setting, bool permanent);
        bool getRSSIByteToggle();

        bool setFixedTransmission(bool Setting, bool permanent);
        bool getFixedTransmission();

        bool setLBT(bool Setting, bool permanent);
        bool getLBT();

        bool setWORCycle(uint8_t WORSetting, bool permanent);
        int getWORCycle();

        bool setEncryptionKey(unsigned char key, bool permanent);

        void printBoardParameters();

        bool setEscapeCharacter(char character);
        uint8_t getEscapeCharacter();

        bool sendTransparentData(String data);
        bool sendTransparentData(uint8_t *data, int size);

        bool sendFixedData(int address, int channel, String data, bool auxAvailable);
        bool sendFixedData(int address, int channel, uint8_t *data, int size, bool auxAvailable);

        String receiveData();
        bool receiveData(uint8_t* data, int size);
};
