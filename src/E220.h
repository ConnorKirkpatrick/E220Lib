//
// Created by connor on 15/11/2021.
//

#ifndef E220LIB_E220_H
#define E220LIB_E220_H


//Define all my constants

#include <Stream.h>
#include <UARTClass.h>

//write data as byte array: command byte, starting address, write-length, parameter
//returns identical, except command byte swaps from c0 to c1
//#define writePermenant 0xC0

//Read data as byte array: command byte, starting address, read-length
//returns identical and then the parameters
//#define read 0xC1

//command byte, starting address, write-length, parameter
//returns identical, except command byte swaps from c0 to c1
//#define writeTemp 0xC2

#define MODE_NORMAL 0			// can send and receive data
#define MODE_WOR_SENDING 1	    // sends a preamble to waken receiver
#define MODE_WOR_RECEIVE 2		// can't transmit, Can only receive from transmitter in mode 1. System only checks for incoming every WOR cycle
#define MODE_PROGRAM 3          // Power saving mode, also used to change the parameters of the device

#define PERMANENT 0xC0          // Will set a given parameter in a non volatile register
#define TEMPORARY 0xC2          // Will set a given parameter in a volatile register

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
#define ADR_38400 0b110		    // 19200 baud
#define ADR_62500 0b110	        // 19200 baud

//Register 3
//sub-packet size setting
#define SPS_200 0b00        //200 bytes
#define SPS_128 0b01        //200bytes
#define SPS_64 0b10         //200bytes
#define SPS_32 0b11         //200bytes

//RSSI Ambient Noise Enable
#define RAN_D 0b00          //disabled (default)
#define RAN_E 0b01          //Enabled

//Transmission Power
#define Power_30 0b00       //30dBm (Default)
#define Power_27 0b01       //27dBm
#define Power_24 0b10       //24dBm
#define Power_21 0b11       //21dBm

//Register 4 reserved for the channel

//Register 5
//RSSI Byte
#define RSSIB_D 0b00        //Disabled (default)
#define RSSIB_E 0b01        //Enabled; after any message is received, module transmits data on TX followed by the RSSI strength byte

//Transmission Mode
#define Transparent 0b00    //Transparent transmission (default)
#define Fixed   0b00        //Fixed transmission; Module will take the first 3 bytes on serial as address high+low+channel

//LBT
#define LBT_D 0b00          //No LBT Monitoring
#define LBT_E 0b01          //LBT Enabled, system will monitor the channel to try to avoid interference when transmitting, can cause delays of upto 2 seconds max

//WOR Cycle
#define FiveHundred 0b00                    //WOR cycle awake every 500ms
#define oneThousand 0b00                    //WOR cycle awake every 1000ms
#define oneThousandFiveHundred 0b00         //WOR cycle awake every 1500ms
#define twoThousand 0b00                    //WOR cycle awake every 2000ms
#define twoThousandFiveHundred 0b00         //WOR cycle awake every 2500ms
#define threeThousand 0b00                  //WOR cycle awake every 3000ms
#define threeThousandFiveHundred 0b00       //WOR cycle awake every 3500ms
#define fourThousand 0b00                   //WOR cycle awake every 4000ms

//Register 6
//cryptoKey high and low, the two halves of the crypto key





class Stream;

class E220 {
    private:

    int _M0;
    int _M1;
    int _AUX;

    Stream* _streamSerial;
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

    bool writeCommand(uint8_t cmdParam, uint8_t address, uint8_t length, uint8_t parameters[]);


    public:

        E220(Stream *s, int PIN_M0, int PIN_M1, int PIN_AUX);

        bool init();
        void setMode(uint8_t mode);
        bool readBoardData();

        bool setAddress(int newAddress, bool permanent);
        uint16_t getAddress();
        bool setBaud(uint8_t newUART, bool permanent);
        uint8_t getBaud();


};


#endif //E220LIB_E220_H
