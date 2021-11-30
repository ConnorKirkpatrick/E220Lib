# Arduino E220 Library

Arduino library made to interface an EBYTE E220 type module.

This library is mainly used for configuration of the modules parameters, sending and receiving data can be done with serial methods

This library was tested with and EBYTE-E220-900T-30D 850MHz-930MHz module

I have included example sketch code to demonstrate how to utilise the Library

---------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Method useage:
#### specific Setters
setAddress(newAdress, boolean) : pass an integer for then new address between 0 and 65535, boolean is true if you want the parameter to be non-voletile

setCryptoKey(newKey, boolean); pass an unsigned char as the key, upto 16bits long, boolean is true if you want the parameter to be non-voletile

#### General Setters
set***(parameter, boolean); pass the constant for the method (see below), boolean is true if you want the parameter to be non-voletile

--------------------------------------------------------------------------------------------------------------------------
## CONSTANT USEAGE:

setBaud()\
UDR_1200 : 1200 baud\
UDR_2400 : 2400 baud\
UDR_4800 : 4800 baud\
UDR_9600 : 9600 baud default\
UDR_19200 : 19200 baud\
UDR_38400 : 34800 baud\
UDR_57600 : 57600 baud\
UDR_115200 : 115200 baud

setParity() (must be the same for both sender and receiver)\
PB_8N1 : 8N1 default\
PB_8O1 : 8O1\
PB_8E1 : 8E1

setAirDataRate() (must be the same for transmitter and receiver)\
ADR_2400 : 2400 baud\
ADR_4800 : 4800 baud\
ADR_9600 : 9600 baud\
ADR_19200 : 19200 baud\
ADR_38400 : 38400 baud\
ADR_62500 : 62500 baud

setSubPacketSize()\
SPS_200 : 200 bytes\
SPS_128 : 128 bytes\
SPS_64 : 64 bytes\
SPS_32 : 32 bytes

SetRSSIAmbient()\
RAN_D : disabled (default)\
RAN_E : Enabled

SetPower\
Power_30 : 30dBm (Default)\
Power_27 : 27dBm\
Power_24 : 24dBm\
Power_21 : 21dBm

setRSSIByteToggle()\
RSSIB_D : Disabled (default)\
RSSIB_E : Enabled; after any message is received, module transmits data on TX followed by the RSSI strength byte

setFixedTransmission()\
Transparent : Transparent transmission (default)\
Fixed : Fixed transmission; Module will take the first 3 bytes on serial as address high+low+channel

setLBT()\
LBT_D : No LBT Monitoring\
LBT_E : LBT Enabled, system will monitor the channel to try to avoid interference when transmitting, can cause delays of upto 2 seconds max

setWORCycle()\
WOR500 : WOR cycle awake every 500ms\
WOR1000 : WOR cycle awake every 1000ms\
WOR1500 : WOR cycle awake every 1500ms\
WOR2000 : WOR cycle awake every 2000ms\
WOR2500 : WOR cycle awake every 2500ms\
WOR3000 : WOR cycle awake every 3000ms\
WOR3500 : WOR cycle awake every 3500ms\
WOR4000 : WOR cycle awake every 4000ms