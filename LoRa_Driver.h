/*
 * LoRa_Driver.h
 *
 *  Created on: Feb 7, 2026
 * 	Author: Matthew Xue
 *
 * 	LoRa is the new radio for the UFC that runs at 410.125 ~ 493.125MHz
 *
 * 	LoRa Pins:
 * 	USART7 GPIO Configuration
 * 	PE8 ------> USART7_TX
 * 	PE7 ------> USART7_RX
 * 	TODO: AUX
 * 	TODO: M0
 * 	TODO: M1
 *
 * 	Datasheet: https://www.cdebyte.com/pdf-down.aspx?id=3820
 */

#ifndef LORA_DRIVER_H_
#define LORA_DRIVER_H_

#include "stm32h7xx_hal.h"
#include "packets.h"
#include "errors.h"

// interrupt priority
#define LORA_IRQN 1

#define LORA_RX_BUFFER_SIZE 256

// Default address values
#define LORA_REG_00_DEFAULT 0xFF // Address filtering disabled
#define LORA_REG_01_DEFAULT 0xFF // Address filtering disabled
#define LORA_REG_02_DEFAULT 0xFF // TODO: what should NETID default be? match ground station radio
#define LORA_REG_03_DEFAULT 0x62 // 9600 Baud, 2.4k Air
#define LORA_REG_04_DEFAULT 0x00 // 240 Byte Packet, RSSI Ambient Noise disabled, 33dBm  transmit power
#define LORA_REG_05_DEFAULT 0x17 // 433.125MH
#define LORA_REG_06_DEFAULT 0x00 // RSSI disabled, transparent mode, disable trunk function, LBT disabled, WOR receiver, WOR bytes disabled
#define LORA_REG_07_DEFAULT 0xFF // TODO: What to do here?
#define LORA_REG_08_DEFAULT 0xFF // TODO: What to do here?


// Pin definitions
// UART Pins
#define LORA_UART_PORT		GPIOE
#define LORA_UART_TX_PIN	GPIO_PIN_8
#define LORA_UART_RX_PIN	GPIO_PIN_7

// Mode Pins
#define LORA_M0_M1_PORT		GPIOE
#define LORA_M0_PIN			GPIO_PIN_11
#define LORA_M1_PIN			GPIO_PIN_12

// AUX Pins
#define LORA_AUX_PORT		GPIOC
#define LORA_AUX_PIN		GPIO_PIN_5

// TODO: reset pin?


#define LORA_REG_MODES_DEFAULT 			0x00




// M0=0,M1=0: transmission mode, serial port open, wireless open, transparent transmission
// M0=0,M1=1: WOR mode, it can be defined as a WOR sender and a WOR receiver
// M0=1,M1=0: Configure mode, user can access registers through the serial port
// M0=1,M1=1: deep hibernation mode, module goes to sleep
enum LORA_MODE {
    LORA_MODE_0_NORMAL        = 0,
    LORA_MODE_1_WOR           = 1,
    LORA_MODE_2_CONFIGURATION = 2,
    LORA_MODE_3_DEEP_SLEEP    = 3
};


// Mode uses M0 and M1 pins to determine the 4 working modes of the LoRa:
// AUX high means the LoRa is ready, low means its not
typedef struct {
    LORA_MODE mode;
    bool AUX;
} LoRa_status;


// bytes 7,6,5 for register 03
// unit: baud
enum LORA_UART_BR {
	LORA_UART_BR_1200   = 0b000,
	LORA_UART_BR_2400   = 0b001,
	LORA_UART_BR_4800   = 0b010,
	LORA_UART_BR_9600   = 0b011,  // default baud rate
	LORA_UART_BR_19200  = 0b100,
	LORA_UART_BR_38400  = 0b101,
	LORA_UART_BR_57600  = 0b110,
	LORA_UART_BR_115200 = 0b111
};

// bytes 2,1,0 for register 03
// unit: k
enum LORA_AIR_RATE {
	LORA_AIR_2400  = 0b010, // default air rate
	LORA_AIR_4800  = 0b011,
	LORA_AIR_9600  = 0b100,
	LORA_AIR_19200 = 0b101,
	LORA_AIR_38400 = 0b110,
	LORA_AIR_62500 = 0b111
};


// bytes 7,6 for register 04
// unit: byte
enum LORA_PACKET_SIZE {
	LORA_PS_240 = 0b00, // default packet size
	LORA_PS_128 = 0b01,
	LORA_PS_64  = 0b10,
	LORA_PS_32  = 0b11
};


// bytes 1,0 for register 04
// unit: dBm
enum LORA_TRANSMIT_POWER {
	LORA_TP_33 = 0b00, // default transmit power, most range/highest power consumption
	LORA_TP_30 = 0b01,
	LORA_TP_27 = 0b10,
	LORA_TP_24 = 0b11
};


// bytes 2,1,0 for register 06
// valid only if byte 3 of register 06 is set to 1, for WOR transmitting mode
// unit: ms
enum LORA_WOR_CYCLE {
	LORA_WOR_500  = 0b000,
	LORA_WOR_1000 = 0b001,
	LORA_WOR_1500 = 0b010,
	LORA_WOR_2000 = 0b011,
	LORA_WOR_2500 = 0b100,
	LORA_WOR_3000 = 0b101,
	LORA_WOR_3500 = 0b110,
	LORA_WOR_4000 = 0b111
};


typedef struct {
	uint8_t reg_00; // Module address high byte
	uint8_t reg_01; // Module address low byte
	uint8_t reg_02; // NETID: This needs to match the ground station radio
	uint8_t reg_03; // Baud rate, air rate
	uint8_t reg_04; // packet size, RSSI Ambient Noise, transmit power
	uint8_t reg_05; // Channel control, calculation: 410.125 + (register value of 0-83)
	uint8_t reg_06; // RSSI, transmission mode, relay function, LBT enables, WOR mode transmit and receive control, WOR cycle
	uint8_t reg_07; // Encryption key high byte
	uint8_t reg_08; // Encryption key low byte

	LoRa_status status;

	LORA_UART_BR baud_rate;
	LORA_AIR_RATE air_rate;
	LORA_PACKET_SIZE packet_size;
	LORA_TRANSMIT_POWER transmit_power;
	LORA_WOR_CYCLE wor_cycle;
} lora_settings_t;

struct RingBuffer {
	uint8_t data[LORA_RX_BUFFER_SIZE];

	// The 'volatile' keyword is CRITICAL here. It tells the compiler:
	// "Hey, these variables change inside an interrupt. Don't cache them!"
	volatile uint16_t head = 0;
	volatile uint16_t tail = 0;

	bool push(uint8_t byte);
	bool pop(uint8_t *byte);
	uint16_t available();
};

struct LoRa_Driver {
	bool isInit = false;
	lora_settings_t settings;
	UART_HandleTypeDef uartStruct = {0};

	UFC_ECODE init();
	UFC_ECODE deinit();

	UFC_ECODE sendCommand(uint8_t* packet, uint8_t pktLen);
	UFC_ECODE tx(uint8_t *input, uint32_t length);

	UFC_ECODE modifyWORCycle_();
	UFC_ECODE modifyTransmitPower();
	UFC_ECODE modifyPacketSize();
	UFC_ECODE modifyAirRate();
	UFC_ECODE modifyBaudRate();

	UFC_ECODE waitForAUX();
	UFC_ECODE setMode(LORA_MODE mode);
};

extern LoRa_Driver LoRaDriver;

// UART7 interrupt function
extern "C" {
	void UART7_IRQHandler(void);
	void LORA_UART_RxCpltCallback(UART_HandleTypeDef *huart);
}


#endif /* LORA_DRIVER_H_ */






