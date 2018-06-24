/*
 * globals.h
 *
 * Created: 26.12.2016 20:21:01
 *  Author: chris
 */ 

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <stdint.h>

#define USE_EEPROM_SETTINGS 1// 1 use eeprom settings, 0 use macros

#define STORE_SETTINGS_TO_EEPROM 0//1 - write new macro configs to eeprom, 0 do nothing

#define	DEBUG_MESSAGES 0//1 ON, 0 OFF
#define NO_LORA_CONNECTED 0 //1 = no lora, 0 = normal
#define DeviceID 0x00
#define MESSAGE_FREQ 0xFF //2 == every 2 seconds, # == every # seconds ...
#define SYNC_WORD 0xA0

#define DESIRED_PACKET_LENGTH 60 //packet size that will be (immediately) sent (smaller packets are filled up with zeros after 10 ms)
#define LORA_PACKET_LENGTH DESIRED_PACKET_LENGTH
#define IN_BUFFER_SIZE 64 //  (SERIAL_BUFFER_SIZE = 64)
#define RING_BUFFER_SIZE 2*IN_BUFFER_SIZE //RF Transmit Ring Buffer size, should be greater as UART Buffer
#define UART_BAUD_RATE 256000
#define LORA_FREQUENCY 433E6
#define LORA_TX_POWER 2 //power in dB, values between 2 and 17
#define LORA_SPREADING_FACTOR 10 //values between 6 and 12, default 7 (with 6 implicit header mode must be used)
#define LORA_SIGNAL_BANDWIDTH 500E3 //bandwith in Hz, default 125E3, supported: 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3 500E3
#define LORA_CODING_RATE 6 //denominator of the coding rate, default 5, allowed x = 5 ... 8 => Coding rate = 4/x
#define LORA_PREAMBLE_LENGTH 6 //preamble length in symbols, defaults to 8, allowed 6 ... 65535
#define LORA_SYNC_WORD  0x55 //byte valued used as sync word, default 0x34
#define LORA_CRC 0 //use crc = 1, no crc = 0 (default)

//Ringbuffer Status
#define NORMAL_WFLIP 0x01
#define WRITE_POINTER_OVERFLOW 0x02
#define FULL_BUFFER_WFLIP 0x04
#define READ_POINTER_OVERFLOW 0x08
#define READ_FLIP_OVERFLOW 0x10

#define XBeeMode 0
#define	RFLoopBackMode 1
#define SerialLoopBackMode 2

#define	LoRa_interface 0
#define Uart_interface 1
	
//Receive States for State machine	
#define WaitForSync 0
#define WaitforCommandandData 1
#define CommandandDataReady 2
#define WaitforCommandandDataRepeat 3

	
//Commands
#define SetDeviceID 0
#define	GetDeviceID 1
#define	SetUARTSyncWord 2
#define	GetUARTSyncWord 3
#define	SetDesiredPacket_Length 4
#define	GetDesiredPacket_Length 5
#define	SetAvrMessageFreq 6
#define	GetAvrMessageFreq 7
#define	SetOperationMode 8
#define	GetOperationMode 9
#define	SetLoraTXPower 10 
#define	GetLoraTXPower 11
#define	SetLoraSpreadingFactor 12
#define	GetLoraSpreadingFactor 13
#define	SetLoraSignalBandwidth 14
#define	GetLoraSignalBandwidth 15
#define	SetLoraCodingRate 16
#define	GetLoraCodingRate 17
#define	SetLoraPreambleLength 18
#define	GetLoraPreambleLength 19
#define	SetLoraSyncWord 20
#define	GetLoraSyncWord 21
#define	SetCRCused 22
#define	GetCRCused 23
#define	SavetoEEPROM 24
#define	GetEEPROMData 25
#define	Get_BeeStatus 26
#define	GetHumanReadableSettinglist 27

//EEPROM Adresses
#define HEX_VALS_START_ADDR 485 //255*2 = 510
#define MESSAGE_ADDR_START_ADDR 25
#define MESSAGE_NUMBER	18
#define MESSAGE_ADDR_AVR_HEADER 25
#define MESSAGE_ADDR_DEVICE_ID 27
#define MESSAGE_ADDR_UART_SYNC_WORD 29
#define MESSAGE_ADDR_BEE_PACKET_L 31
#define MESSAGE_ADDR_MESSAGE_FREQUENCY 33
#define MESSAGE_ADDR_OPERATION_MODE 35
#define MESSAGE_ADDR_XBEE_MODE 37
#define MESSAGE_ADDR_RF_LOOPBACK_MODE 39
#define MESSAGE_ADDR_SERIAL_LOOPBACK_MODE 41
#define MESSAGE_ADDR_UNKOWN_MODE 43
#define MESSAGE_ADDR_LORA_HEADER  45
#define MESSAGE_ADDR_TX_POWER 47
#define MESSAGE_ADDR_SPREADING_FACTOR 49
#define MESSAGE_ADDR_SIG_BANDWIDTH 51
#define MESSAGE_ADDR_CODING_RATE 53
#define MESSAGE_ADDR_PREAMBLE_LENGTH 55
#define MESSAGE_ADDR_LORA_SYNC_WORD 57
#define MESSAGE_ADDR_LORA_CRC_USED 59

#endif /* GLOBALS_H_ */
