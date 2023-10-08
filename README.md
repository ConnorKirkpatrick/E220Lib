# Arduino E220 Library

Arduino library made to interface an EBYTE E220 type module.

This library is mainly used for configuration of the modules parameters, sending and receiving data can be done with serial methods

This library was tested with and EBYTE-E220-900T-30D 850MHz-930MHz module

I have included example sketch code to demonstrate how to utilise the Library

----------------------------------------------------------------------------------------------------------------------
##Usage

##Constants
The radio module requires 3 provided constants, and 2 optional constants that enhance the capabilities of the module.
* (Required) Serial: You must provide the constructor with a pointer to either a hardware or software serial that the module is connected to
* (Required) M0: You must provide the digital pin connected to the modules M0 pin
* (Required) M1: You must provide the digital pin connected to the modules M1 pin
* (Optional) AUX: You may provide the digital pin connected to the modules Aux pin
* (Optional) escapeCharacter: You may provide an escape character to use with the builtin send & receive functions. Do this using the setEscapeCharacter() function
##Configuration
To configure the module, you must use the builtin modules of this library. Most of the values needed will either be contained inside the constants' section below or simply be normal values<br>
When setting an option, there will be a boolean parameter you must provide. This value will determine if the configuration option is to be saved permanently or simply until power off.
##Radio communication
###Communication types
The E220 Module has 2 main modes of communication; Transparent transmission or Fixed transmission.<br>
In both modes of transmission, the contents of the message to send must be at least 9 bytes.
####Transparent Transmission
When in transparent transmission, the module will send data to devices with the same address and channel. You may also specify the broadcast address(0xffff) to send to all other devices on the same channel.<br>
In this mode, we are limited to communicating with devices on the same channel as us.
###Fixed transmission
When in fixed transmission, the first 3 bytes of data send to the module are read as the address and channel of the device we wish to communicate with.<br>
We can still use a broadcast address to communicate with all devices on the targeted channel at the same time, but we now can message any address on any channel as we wish.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------
## Configuration methods:
E220(Serial_ *s, int PIN_M0, int PIN_M1, int PIN_AUX): The constructor for the radio object. Provide the serial object in the form like "Serial_ &mySerial = (Serial_ &)Serial2;"<br> The serial provided may be a software serial type.

SetAddress(address): Simply provide an integer in range 0-65535 as the address

SetChannel(channel): Simply provide an integer in range 0-80 as the channel

setBaud(newBaudRate) This is the baud rate to communicate over serial with the arduino\
UDR_1200 : 1200 baud\
UDR_2400 : 2400 baud\
UDR_4800 : 4800 baud\
UDR_9600 : 9600 baud (default)\
UDR_19200 : 19200 baud\
UDR_38400 : 34800 baud\
UDR_57600 : 57600 baud\
UDR_115200 : 115200 baud

setParity(newParity) Parity bit for serial communication(must be the same for both sender and receiver)\
PB_8N1 : 8N1 (default)\
PB_8O1 : 8O1\
PB_8E1 : 8E1

setAirDataRate(airDataRate) Think of this as the baud rate between the radio modules (must be the same for transmitter and receiver)\
ADR_2400 : 2400 baud (default)\
ADR_4800 : 4800 baud\
ADR_9600 : 9600 baud\
ADR_19200 : 19200 baud\
ADR_38400 : 38400 baud\
ADR_62500 : 62500 baud

setSubPacketSize(newPacketSize)\
SPS_200 : 200 bytes (default)\
SPS_128 : 128 bytes\
SPS_64 : 64 bytes\
SPS_32 : 32 bytes

SetRSSIAmbient(newRAN) This option will enable you to view environmental noise via the E220 module(see the documentation)\
RAN_D : disabled (default)\
RAN_E : Enabled

SetPower(newPower) This option changes the broadcast power of the module. Higher powers user more energy, but it is not linear as the voltage converter is more efficient at higher values\
Power_30 : 30dBm (Default)\
Power_27 : 27dBm\
Power_24 : 24dBm\
Power_21 : 21dBm

setRSSIByteToggle(toggle) When enabled, the module will add an RSSI value to the end of any received data\
RSSIB_D : Disabled (default)\
RSSIB_E : Enabled

setFixedTransmission(toggle) This option swaps between fixed and transparent transmission as discussed in the Communication types section above\
Transparent : Transparent transmission (default)\
Fixed : Fixed transmission

setLBT(newLBT) When enabled the system will monitor the channel to try to avoid interference when transmitting, can cause delays of upto 2 seconds \
LBT_D : No LBT Monitoring\
LBT_E : LBT Enabled 

setWORCycle(newWORSetting)\
WOR500 : WOR cycle awake every 500ms\
WOR1000 : WOR cycle awake every 1000ms\
WOR1500 : WOR cycle awake every 1500ms\
WOR2000 : WOR cycle awake every 2000ms\
WOR2500 : WOR cycle awake every 2500ms\
WOR3000 : WOR cycle awake every 3000ms\
WOR3500 : WOR cycle awake every 3500ms\
WOR4000 : WOR cycle awake every 4000ms

--------------------------------------------------------------------------------------------------------------------------
## Communication methods:
When the module is configured properly, you can simply use the builtin serial Read() and Write() methods to send and receive data.<br>
I have provided a few functions built into the library to assist you in doing this:

setEscapeCharacter(char): This method is used to set the escape character of transmissions. This is the value that the radio will read upto before returning data if using methods that return a string<br>
getEscapeCharacter(): Used to check what escape character you are using.<br>
If this character is set to "", your minimum message size rises to 10 bytes from 9.

sendTransparentData(String): This method is used to send data when in transparent transmission mode. This method will attach the escape character to the end of your transmission<br>
sendTransparentData(data,size): This method sends a byte array rather than a string.

sendFixedData(address,channel,String, flag):
This method is used to send data when in fixed mode. You must provide an integer for both the address and channel, along with a String of data.\
The flag is used to configure timing control via the aux pin. If enabled, the device will ensure that the last broadcast is complete before sending a new packet, allowing for rapid channel or address changes.<br>
If this is disabled, there is a chance that the next message will be appended to the first message is there is too little of a delay between the broadcasts.\
This method will append the escape character to the end of your message.<br>

sendFixedData(address,channel,data, size, flag): This method sends a byte array rather than a string.<br>

receiveData(): A simple method used to check if data is received, and if so return it as a string. It will read upto the defined escapeCharacter.\
receiveData(array,size): This method will read a specified amount of bytes into your provided array. It will not search for the escape character when reading. This will return true if data is read, false if not.



