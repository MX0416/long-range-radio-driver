/*
 * LoRa_Driver.cpp
 *
 *  Created on: Feb 7, 2026
 *      Author: Matthew Xue
 */

#include "LoRa_Driver.h"
#include "terminal.h"


UFC_ECODE LoRa_Driver::waitForAUX() {
	uint32_t startTick = HAL_GetTick();
	while (HAL_GPIO_ReadPin(LORA_AUX_PORT, LORA_AUX_PIN) == GPIO_PIN_RESET) {
		if ((HAL_GetTick() - startTick) > 500) {
			return UFC_ECODE_FAIL;
		}
	}
	HAL_Delay(2);
	return UFC_ECODE_OK;
}

UFC_ECODE LoRa_Driver::setMode(LORA_MODE mode) {
	switch(mode) {
		case LORA_MODE_0_NORMAL: // M1=0, M0=0
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
			break;

		case LORA_MODE_1_WOR: // M1=0, M0=1
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M0_PIN, GPIO_PIN_SET);
			break;

		case LORA_MODE_2_CONFIGURATION: // M1=1, M0=0
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
			break;

		case LORA_MODE_3_DEEP_SLEEP: // M1=1, M0=1
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LORA_M0_M1_PORT, LORA_M0_PIN, GPIO_PIN_SET);
			break;
	}

	settings.status.mode = mode;
	return waitForAUX();
}

LoRa_status default_status = {
	.mode = LORA_MODE_0_NORMAL,
	.AUX = true
};

// create driver instance with default settings
lora_settings_t LoRaDefaultSettings = {
	.reg_00 = LORA_REG_00_DEFAULT,
	.reg_01 = LORA_REG_01_DEFAULT,
	.reg_02 = LORA_REG_02_DEFAULT,
	.reg_03 = LORA_REG_03_DEFAULT,
	.reg_04 = LORA_REG_04_DEFAULT,
	.reg_05 = LORA_REG_05_DEFAULT,
	.reg_06 = LORA_REG_06_DEFAULT,
	.reg_07 = LORA_REG_07_DEFAULT,
	.reg_08 = LORA_REG_08_DEFAULT,

	.status = default_status,

	.baud_rate = LORA_UART_BR_9600,
	.air_rate = LORA_AIR_2400,
	.packet_size = LORA_PS_240,
	.transmit_power = LORA_TP_33,
	.wor_cycle = LORA_WOR_500
};

LoRa_Driver LoRaDriver;

// 2 bytes buffer to read to in the interrupt function
static uint8_t UART_ISR_Rx_data[2];


UFC_ECODE LoRa_Driver::init() {
	terminal.uartPrintf("Initializing LoRa...\r\n");

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	settings = LoRaDefaultSettings;

	__HAL_RCC_UART7_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStruct.Pin = LORA_UART_TX_PIN | LORA_UART_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
	HAL_GPIO_Init(LORA_UART_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LORA_M0_PIN | LORA_M1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LORA_M0_M1_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LORA_AUX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_AUX_PORT, &GPIO_InitStruct);

	// AUX pin will be low until LoRa is ready
	if (waitForAUX() != UFC_ECODE_OK) {
		terminal.uartPrintf("LoRa AUX pin stuck on low during init, terminating....\r\n");
		return UFC_ECODE_FAIL;
	}

	// Hard Coded UART Settings
	uartStruct.Instance = UART7;
	uartStruct.Init.WordLength = UART_WORDLENGTH_8B;
	uartStruct.Init.StopBits = UART_STOPBITS_1;
	uartStruct.Init.Parity = UART_PARITY_NONE;
	uartStruct.Init.BaudRate = 9600; // default LoRa baud rate
	uartStruct.Init.Mode = UART_MODE_TX_RX;
	uartStruct.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uartStruct.Init.OverSampling = UART_OVERSAMPLING_16;
	uartStruct.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	uartStruct.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uartStruct.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&uartStruct) != HAL_OK) {
		return UFC_ECODE_FAIL;
	}
	uartStruct.RxCpltCallback = LORA_UART_RxCpltCallback;

	// Interrupt initialization
	NVIC_SetPriority(UART7_IRQn, LORA_IRQN);
	HAL_NVIC_EnableIRQ(UART7_IRQn);



	// Command structure: command + start address + length + parameters
	uint8_t commandPacket[] {
		0xC0, // Set Registers command
		0x00, // Start Address: Register 0
		0x09, // Length: writing all 9 registers

		LoRaDefaultSettings.reg_00,
		LoRaDefaultSettings.reg_01,
		LoRaDefaultSettings.reg_02,
		LoRaDefaultSettings.reg_03,
		LoRaDefaultSettings.reg_04,
		LoRaDefaultSettings.reg_05,
		LoRaDefaultSettings.reg_06,
		LoRaDefaultSettings.reg_07,
		LoRaDefaultSettings.reg_08
	};

	// TODO: whoami

	if (sendCommand(commandPacket, sizeof(commandPacket)) == UFC_ECODE_OK) {

		// Start UART interrupts
		__HAL_UART_CLEAR_FLAG(&uartStruct, UART_CLEAR_OREF);
		__HAL_UART_FLUSH_DRREGISTER(&uartStruct);

		if (HAL_UART_Receive_IT(&uartStruct, UART_ISR_Rx_data, 1) != HAL_OK) {
			return UFC_ECODE_FAIL;
		}

		isInit = true;
		return UFC_ECODE_OK;
	} else {
		return UFC_ECODE_FAIL;
	}
}

UFC_ECODE LoRa_Driver::sendCommand(uint8_t* packet, uint8_t pktLen) {
	terminal.uartPrintf("Sending command packet\r\n");
	terminal.uartPrintf("Switching to configuration mode...\r\n");
//	setMode(LORA_MODE_2_CONFIGURATION);
	if (setMode(LORA_MODE_2_CONFIGURATION) != UFC_ECODE_OK) {
	        terminal.uartPrintf("Error: Could not enter Config Mode (AUX stuck LOW)\r\n");
	        return UFC_ECODE_FAIL;
	}


	// testing
//	if (streq(argv[1], "auxtest")) {
//	    GPIO_PinState state = HAL_GPIO_ReadPin(LORA_AUX_PORT, LORA_AUX_PIN);
//	    addResponse("AUX Pin is currently: %s\r\n", (state == GPIO_PIN_SET) ? "HIGH (Idle)" : "LOW (Busy)");
//	    terminal.uartPrintf("Switching to configuration mode...\r\n");
//	    return UFC_ECODE_OK;
//	}

	if (pktLen < 3) {
		return UFC_ECODE_FAIL;
	}

	if (HAL_UART_Transmit(&uartStruct, packet, pktLen, 100) != HAL_OK) {
		return UFC_ECODE_FAIL;
	}

	uint8_t response[12] = {0};
	if (HAL_UART_Receive(&uartStruct, response, pktLen, 500) == HAL_OK) {
		if (response[0] == 0xC1) {
			terminal.uartPrintf("Packet sent! Returning to normal mode.\r\n");
			setMode(LORA_MODE_0_NORMAL);
			return UFC_ECODE_OK;
		}
	}
	terminal.uartPrintf("Packet failed to send. Returning to normal mode.\r\n");
	setMode(LORA_MODE_0_NORMAL);
	return UFC_ECODE_FAIL;
}

UFC_ECODE LoRa_Driver::tx(uint8_t *input, uint32_t length) {
	if (!isInit) {
		terminal.uartPrintf("failed 1\r\n");
		return UFC_ECODE_FAIL;
	}
	if (settings.status.mode != LORA_MODE_0_NORMAL) {
		terminal.uartPrintf("TX failed, radio not in normal mode\r\n");
		return UFC_ECODE_FAIL;
	}
	if (waitForAUX() != UFC_ECODE_OK) {
		terminal.uartPrintf("failed 2\r\n");
		return UFC_ECODE_FAIL;
	}
	if (HAL_UART_Transmit(&uartStruct, input, length, 3 * length) != HAL_OK) {
		terminal.uartPrintf("failed 3\r\n");
		return UFC_ECODE_FAIL;
	}
	terminal.uartPrintf("Testing tx....\r\n");
	return UFC_ECODE_OK;
}

void LORA_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART7) {
		uint8_t incomingByte = UART_ISR_Rx_data[0];

		// TODO: what do i do with the data


		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
		__HAL_UART_FLUSH_DRREGISTER(huart);
		HAL_UART_Receive_IT(huart, UART_ISR_Rx_data, 1);
	}
}

void UART7_IRQHandler(void) {
	HAL_UART_IRQHandler(&LoRaDriver.uartStruct);
}

