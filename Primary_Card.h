/*
 *  Primary_Card.h
 *
 *  Created on: Jul 4, 2024
 *  Author: Ryan Srichai
 *  Notes: This is the "main.h" for the primary card project.
 */

#ifndef PRIMARY_CARD_H_
#define PRIMARY_CARD_H_

/* common includes */
#include "common_init.h"
#include "packets.h"
#include "terminal.h"
#include "phone.h"
#include "W25N512GV_Driver.h"
/* primary card includes */
#include "BNO055_Driver.h"
#include "LoRa_Driver.h"
#include "GPS_Driver.h"
#include "BMP390_Driver.h"

#define INIT_ATTEMPTS 3

/* RFD sends every X main loop cycles */
// TODO: original was 5, testing
#define LORA_SEND_CYCLE 10

/* main function for primary card */
void Primary_Card();

/* initialise primary card utilities */
void Primary_Card_Init();

/* update status packet for primary card */
void updateStatus();

/* automates the process of requesting a packet and receiving data */
UFC_ECODE requestPacket(uint16_t card_id, uint8_t packetType);

/* function that collects sensor data locally */
UFC_SENSOR pollSensors();

/* function that does rocket state specific operations */
void stateSpecific(UFC_SENSOR sensorUpdate);

/* send local packets via RFD */
void primaryBroadcast(UFC_SENSOR sensorUpdate);

/* collect packets from secondary card over CAN and send through RFD */
void secondaryBroadcast();

/* collect packets from the pitot card over CAN and send through RFD */
void pitotBroadcast();

#endif /* PRIMARY_CARD_H_ */
