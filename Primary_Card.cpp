#include "Primary_Card.h"

uint32_t tickCount = 0;
uint32_t launchTime = 0xFFFFFFFF;
bool telemetry = true;

/* this is the estimated time in ms from launch to land to write the circular buffer to the flash
This is an overestimation to ensure that the circular buffer contents are safely written to the flash
even if landing is not detected
*/
#define SAFE_LANDING_TIME_MS  3600000 // 1 hour

void Primary_Card() {
	RESTART: // for restart command
	restart = 0;
	InitAlways();
	/* delay has shown to experimentally improve reliability */
	HAL_Delay(500);
	/* Primary card specific initialisation */
	Primary_Card_Init();

	/* main loop */
	while (1) {
		updateStatus();
		UFC_SENSOR sensorUpdate = pollSensors();
		stateSpecific(sensorUpdate);
		terminal.tick();
		if (restart) {
			goto RESTART;
		}
		tickCount++;
	}
}

void Primary_Card_Init() {
	/* init status */
	uwTick = 0;
	statusPacket.card_id = CARD_ID;
	statusPacket.card_type = CARD_TYPE;
	statusPacket.card_redirect = 0;
	statusPacket.init_status = 0;
	statusPacket.timestamp_offset = 0;
	/* init state detector */
	StateDetector.setAccelHertz(100); // BNO has 100 hertz rate
	StateDetector.setAltHertz(100); // altimeter has around 100 hertz rate
	/* init led and play primary card startup sequence */
	LedDriver.init();
	LedDriver.startupPrimary();
	LedDriver.blink(LED_BLUE);
	/* init terminal */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(terminal.init())) {
			statusPacket.init_status |= 0b1;
			break;
		}
	}
	/* init phone */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(phone.init())) {
			terminal.uartTx("phone successfully initialised\r\n");
			statusPacket.init_status |= 0b10;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("phone could not initialise\r\n");
			}
		}
	}
	/* init FlashDriver */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(FlashDriver.init())) {
			terminal.uartTx("FlashDriver successfully initialised\r\n");
			statusPacket.init_status |= 0b100;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("FlashDriver could not initialise\r\n");
			}
		}
	}
	/* init CircularBuffer */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(CircularBuffer.init(CIRCLE_BUFFER))) {
			terminal.uartTx("CircularBuffer successfully initialised\r\n");
			statusPacket.init_status |= 0b1000;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("CircularBuffer could not initialise\r\n");
			}
		}
	}
	/* init BnoDriver */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(BnoDriver.init())) {
			terminal.uartTx("BnoDriver successfully initialised\r\n");
			statusPacket.init_status |= 0b10000;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("BnoDriver could not initialise\r\n");
			}
		}
	}
	/* init AltDriver */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(AltDriver.init())) {
			terminal.uartTx("Altimeter successfully initialised\r\n");
			statusPacket.init_status |= 0b100000;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("Altimeter could not initialise\r\n");
			}
		}
	}
	/* init GPS */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(GpsDriver.init())) {
			terminal.uartTx("GPS successfully initialised\r\n");
			statusPacket.init_status |= 0b1000000;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("GPS could not initialise\r\n");
			}
		}
	}
	/* init LoRa */
	for (uint16_t i = 0; i < INIT_ATTEMPTS; i++) {
		if (isOK(LoRaDriver.init())) {
			terminal.uartTx("LoRa successfully initialised\r\n");
			statusPacket.init_status |= 0b10000000;
			break;
		} else {
			if (i == INIT_ATTEMPTS - 1) {
				terminal.uartTx("LoRa could not initialise\r\n");
			}
		}
	}
	LedDriver.unblink();
	LedDriver.lightStatus();
}

void updateStatus() {
	statusPacket.header.timestamp = HAL_GetTick();
	statusPacket.header.state = StateDetector.rocket_state;
	statusPacket.card_redirect = terminal.routeInput;
}

UFC_SENSOR pollSensors() {
	UFC_SENSOR output = UFC_SENSOR_NULL;
	UFC_ECODE status = UFC_ECODE_OK;
	float bnoAccelData[3];
	status = BnoDriver.readAccel(bnoAccelData);
	bnoPacket.accel_x = bnoAccelData[0];
	bnoPacket.accel_y = bnoAccelData[1];
	bnoPacket.accel_z = bnoAccelData[2];
	if (isOK(status)) {
		output |= UFC_SENSOR_BNO_ACCEL;
	}
	float bnoGyroData[3];
	status = BnoDriver.readGyro(bnoGyroData);
	bnoPacket.gyro_x = bnoGyroData[0];
	bnoPacket.gyro_y = bnoGyroData[1];
	bnoPacket.gyro_z = bnoGyroData[2];
	if (isOK(status)) {
		output |= UFC_SENSOR_BNO_GYRO;
	}
	float bnoEulData[3];
	status = BnoDriver.readEul(bnoEulData);
	bnoPacket.eul_heading = bnoEulData[0];
	bnoPacket.eul_roll = bnoEulData[1];
	bnoPacket.eul_pitch = bnoEulData[2];
	if (isOK(status)) {
		output |= UFC_SENSOR_BNO_EUL;
	}
	bnoPacket.header.timestamp = HAL_GetTick() + statusPacket.timestamp_offset;
	bnoPacket.header.state = StateDetector.rocket_state;
	if (GpsDriver.getBufferLevel() > 0) {
		status = GpsDriver.getNavSolution(&gpsPacket);
		if (isOK(status)) {
			output |= UFC_SENSOR_GPS;
		}
	}
	gpsPacket.header.timestamp = HAL_GetTick() + statusPacket.timestamp_offset;
	gpsPacket.header.state = StateDetector.rocket_state;
	float altData[2];
	status = AltDriver.readData(altData);
	altPacket.temperature = altData[0];
	altPacket.pressure = altData[1];
	if (isOK(status)) {
		output |= UFC_SENSOR_TEMPERATURE | UFC_SENSOR_PRESSURE;
	}
	altPacket.header.timestamp = HAL_GetTick() + statusPacket.timestamp_offset;
	altPacket.header.state = StateDetector.rocket_state;
	return output;
}

void stateSpecific(UFC_SENSOR sensorUpdate) {
	/* DETECT STATE */
	if (isBNO_ACCEL(sensorUpdate)) {
		StateDetector.recordAccel(magnitude(bnoPacket.accel_x, bnoPacket.accel_y, bnoPacket.accel_z));
	}
	if (isALT(sensorUpdate)) {
		StateDetector.recordAlt(altPacket.pressure);
	}

	if (StateDetector.rocket_state != StateDetector.old_state) {
		/* transition from one state to another - this can be done via command or through the stateDetector */
		if (StateDetector.rocket_state == UFC_STATE_PAD) {
			LedDriver.unblink();
			LedDriver.lightStatus();
		}
		if (StateDetector.rocket_state == UFC_STATE_PAD_RECORD) {
			LedDriver.blink(LED_YELLOW);
		}
		if (StateDetector.rocket_state == UFC_STATE_POWERED_ASCENT) {
			launchTime = HAL_GetTick();
			StateDetector.launch();
			LedDriver.unblink();
			LedDriver.killAll();
		}
		if (StateDetector.rocket_state == UFC_STATE_UNPOWERED_ASCENT) {
			LedDriver.killAll();
			LedDriver.light(LED_RED);
		}
		if (StateDetector.rocket_state == UFC_STATE_FREE_FALL) {
			LedDriver.killAll();
			LedDriver.light(LED_GREEN);
		}
		if (StateDetector.rocket_state == UFC_STATE_DROGUE_CHUTE) {
			LedDriver.killAll();
			LedDriver.light(LED_BLUE);
		}
		if (StateDetector.rocket_state == UFC_STATE_MAIN_CHUTE) {
			LedDriver.killAll();
			LedDriver.light(LED_YELLOW);
		}
		if (StateDetector.rocket_state == UFC_STATE_LANDED) {
			StateDetector.land();
			LedDriver.unblink();
			LedDriver.lightAll();
			CircularBuffer.transferToFlash();
		}
	}
	StateDetector.old_state = StateDetector.rocket_state;
	if (StateDetector.rocket_state == UFC_STATE_PAD) {
		/* write to circular buffer */
		if (isBNO(sensorUpdate)) {
			CircularBuffer.write((uint8_t *) &bnoPacket, sizeof(bnoPacket_t));
		}
		if (isGPS(sensorUpdate)) {
			CircularBuffer.write((uint8_t *) &gpsPacket, sizeof(gpsPacket_t));
		}
		if (isALT(sensorUpdate)) {
			CircularBuffer.write((uint8_t *) &altPacket, sizeof(altPacket_t));
		}
	} else if (StateDetector.rocket_state == UFC_STATE_PAD_RECORD || StateDetector.rocket_state == UFC_STATE_POWERED_ASCENT ||
	    StateDetector.rocket_state == UFC_STATE_UNPOWERED_ASCENT || StateDetector.rocket_state == UFC_STATE_FREE_FALL ||
	    StateDetector.rocket_state == UFC_STATE_DROGUE_CHUTE || StateDetector.rocket_state == UFC_STATE_MAIN_CHUTE ||
	    StateDetector.rocket_state == UFC_STATE_LANDED) {
		/* write packets to flash */
		UFC_ECODE flashStatus = UFC_ECODE_OK;
		if (isBNO(sensorUpdate)) {
			flashStatus |= FlashDriver.writeNext((uint8_t *) &bnoPacket, sizeof(bnoPacket_t));
		}
		if (isGPS(sensorUpdate)) {
			flashStatus |= FlashDriver.writeNext((uint8_t *) &gpsPacket, sizeof(gpsPacket_t));
		}
		if (isALT(sensorUpdate)) {
			flashStatus |= FlashDriver.writeNext((uint8_t *) &altPacket, sizeof(altPacket_t));
		}
		if (CircularBuffer.transferFlag == false && isFULL(flashStatus)) {
			CircularBuffer.transferToFlash();
		}
	}
	/* all stages code */
	if (CircularBuffer.transferFlag == false &&  (int64_t) HAL_GetTick() - launchTime > SAFE_LANDING_TIME_MS) {
		CircularBuffer.transferToFlash();
	}
	/* send via LoRa */
	if (tickCount % LORA_SEND_CYCLE == 0) {
		primaryBroadcast(sensorUpdate);
		secondaryBroadcast();
		pitotBroadcast();
	}
	StateDetector.getState();
}

void primaryBroadcast(UFC_SENSOR sensorUpdate) {
	if (isBNO(sensorUpdate)) {
		if (telemetry) {
			LoRaDriver.tx((uint8_t *) &bnoPacket, sizeof(bnoPacket_t));
		}
	}
	if (isGPS(sensorUpdate)) {
		if (telemetry) {
			LoRaDriver.tx((uint8_t *) &gpsPacket, sizeof(gpsPacket_t));
		}
	}
	if (isALT(sensorUpdate)) {
		if (telemetry) {
			LoRaDriver.tx((uint8_t *) &altPacket, sizeof(altPacket_t));
		}
	}
}

void secondaryBroadcast() {
	uint16_t *secondaryCards = phone.getCardIDs(SECONDARY_CARD_TYPE);
	if (secondaryCards[0]) {
		phone.requestPacket(secondaryCards[1], SENSOR_PKT);
		if (telemetry) {
			LoRaDriver.tx((uint8_t *) &sensorPacket, sizeof(sensorPacket_t));
		}
	}
}

void pitotBroadcast() {
	uint16_t *pitotCards = phone.getCardIDs(PITOT_CARD_TYPE);
	if (pitotCards[0]) {
		phone.requestPacket(pitotCards[1], PITOT_CENTER_PKT);
		phone.requestPacket(pitotCards[1], PITOT_RADIAL_PKT);
		if (telemetry) {
			LoRaDriver.tx((uint8_t *) &pitotCenterPacket, sizeof(pitotCenterPacket_t));
			LoRaDriver.tx((uint8_t *) &pitotRadialPacket, sizeof(pitotRadialPacket_t));
		}
	}
}

/* add primary card terminal commands here */
UFC_ECODE Terminal_Driver::CardTerminalHandler(uint8_t argc, char argv[TERMINAL_ARGS_LIMIT][TERMINAL_ARG_LENGTH_LIMIT]) {
	if (argc == 1 && streq(argv[0], "help")) {
		addResponse("Common Commands:\r\n");
		addResponse("    help - displays this message\r\n");
		addResponse("    whoami - displays CARD_ID and CARD_TYPE\r\n");
		addResponse("    status {CARD_ID} - displays card status\r\n");
		addResponse("    echo {string} - prints back the input string\r\n");
		addResponse("    led on # - turns on the led corresponding to # (1-4)\r\n");
		addResponse("    led on all - turns on all the leds\r\n");
		addResponse("    led off # - turns off the led corresponding to # (1-4)\r\n");
		addResponse("    led off all - turns off all the leds\r\n");
		addResponse("    pwd - displays CARD_TYPE\r\n");
		addResponse("    ls - displays CARD_ID and CARD_TYPE of all cards in the network\r\n");
		addResponse("    cd {CARD_ID} - switch to another card\r\n");
		addResponse("    sync - re-queries the network\r\n");
		addResponse("    sync timestamp - syncs all card's packet timestamp to this card's timestamp\r\n");
		addResponse("    packets - displays all packet types and sizes");
		addResponse("    restart - reboots this card\r\n");
		addResponse("    LAUNCH - manually triggers the launch sequence\r\n");
		addResponse("    LAND - manually triggers the land sequence\r\n");
		addResponse("    RESET - reset all card states to STATE_PAD and erases all flashes\r\n");
		addResponse("Flash Commands:\r\n");
		addResponse("    flash status - displays status information from the flash chip\r\n");
		addResponse("    flash init - reinitialises the flash\r\n");
		addResponse("    flash erase - erases all data on the flash\r\n");
		addResponse("    flash whoami - issues the whoami command to the flash chip over QUADSPI\r\n");
		addResponse("    flash registers - displays register values for the flash chip\r\n");
		addResponse("    flash restart - restarts the flash chip\r\n");
		addResponse("    flash read {address} - reads and prints 512 bytes from the specified address\r\n");
		addResponse("    flash write {data} - writes ASCII text to flash at writePointer\r\n");
		addResponse("    flash writePacket {packet} - writes packet to flash chip at writePointer\r\n");
		addResponse("    flash force write {data} {address} - writes ASCII text to flash at address\r\n");
		addResponse("Primary Card Commands:\r\n");
		addResponse("    speedtest - does a one second speedtest for all card operations\r\n");
		addResponse("    sensor speedtest - does a one second speedtest for all card sensors\r\n");
		addResponse("    bno - displays snapshot BNO055 data\r\n");
		addResponse("    bno init - reinitialises the BNO055\r\n");
		addResponse("    bno raw - displays raw BNO055 data\r\n");
		addResponse("    bno temp - displays temperature value from the BNO055\r\n");
		addResponse("    bno whoami - issues the chip_id command to the BNO055 over I2C\r\n");
		addResponse("    bno registers - displays register values for the BNO055\r\n");
		addResponse("    bno speedtest - does a one second datarate test of the BNO055\r\n");
		addResponse("    alt - displays snapshot BMP390 data\r\n");
		addResponse("    alt init - reinitialises the BMP390\r\n");
		addResponse("    alt raw - displays snapshot BMP390 raw data\r\n");
		addResponse("    alt whoami - issues the chip_id command to the BMP390 over I2C\r\n");
		addResponse("    alt registers - displays register values for the BMP390\r\n");
		addResponse("    alt speedtest - does a one second datarate test of the BMP390\r\n");
		addResponse("    gps - displays a snapshot of current GPS data\r\n");
		addResponse("    gps init - reinitialises the GPS\r\n");
		addResponse("    gps whoami - displays the software and hardware version of the GPS\r\n");
		addResponse("    gps buffer - gets the number of pending output bytes from the GPS\r\n");
		addResponse("    gps speedtest - does a one second datarate test of the GPS\r\n");
		addResponse("    lora init - reinitialises the LoRa\r\n");
		addResponse("    lora <command> - runs a command on the LoRa\r\n");
		addResponse("    lora help - displays available LoRa commands\r\n");
		addResponse("    lora send <text> - sends ASCII text over LoRa\r\n");
		addResponse("    i2c restart - attempted recovery of I2C bus\r\n");
		return UFC_ECODE_OK;
	}
	if (argc == 2 && streq(argv[0], "tele") && streq(argv[1], "on")) {
		telemetry = true;
		addResponse("live telemetry turned on\r\n");
		return UFC_ECODE_OK;
	}
	if (argc == 2 && streq(argv[0], "tele") && streq(argv[1], "off")) {
		telemetry = false;
		addResponse("live telemetry turned off\r\n");
		return UFC_ECODE_OK;
	}
	if (argc == 1 && streq(argv[0], "speedtest")) {
		uint32_t initMs = HAL_GetTick();
		uint32_t mainLoopCount = 0;
		while (HAL_GetTick() - initMs < 1000) {
			updateStatus();
			UFC_SENSOR sensorUpdate = pollSensors();
			stateSpecific(sensorUpdate);
			terminal.tick();
			tickCount++;
			mainLoopCount++;
		}
		addResponse("Main loop executed: %d times in 1 second\r\n", mainLoopCount);
		return UFC_ECODE_OK;
	}
	if (argc == 2 && streq(argv[0], "sensor") && streq(argv[1], "speedtest")) {
		uint32_t initMs = HAL_GetTick();

		UFC_SENSOR sensorStatus = UFC_SENSOR_NULL;
		uint32_t successCount = 0;
		uint32_t accelSuccess = 0;
		uint32_t eulSuccess = 0;
		uint32_t gyroSuccess = 0;
		uint32_t gpsSuccess = 0;
		uint32_t altSuccess = 0;
		uint32_t failCount = 0;

		while (HAL_GetTick() - initMs < 1000) {
			sensorStatus = pollSensors();

			if (isBNO_ACCEL(sensorStatus) && isBNO_EUL(sensorStatus) && isBNO_GYRO(sensorStatus) && isGPS(sensorStatus) && isALT(sensorStatus)) {
				successCount++;
			} else {
				failCount++;
			}
			if (isBNO_ACCEL(sensorStatus)) {
				accelSuccess++;
			}
			if (isBNO_EUL(sensorStatus)) {
				eulSuccess++;
			}
			if (isBNO_GYRO(sensorStatus)) {
				gyroSuccess++;
			}
			if (isGPS(sensorStatus)) {
				gpsSuccess++;
			}
			if (isALT(sensorStatus)) {
				altSuccess++;
			}
		}
		addResponse("Total packets received: %d\r\n", successCount + failCount);
		addResponse("Successes: %d\r\n", successCount);
		addResponse(" - bno_accel: %d\r\n", accelSuccess);
		addResponse(" - bno_eul: %d\r\n", eulSuccess);
		addResponse(" - bno_gyro: %d\r\n", gyroSuccess);
		addResponse(" - gps: %d\r\n", gpsSuccess);
		addResponse(" - alt: %d\r\n", altSuccess);
		addResponse("Failures: %d\r\n", failCount);
		addResponse("Success rate: %d\r\n",  successCount * 100 / (failCount + successCount));
		return UFC_ECODE_OK;
	}
	if (streq(argv[0], "bno")) {
		if (argc == 1) {
			float bnoAccel[3];
			BnoDriver.readAccel(bnoAccel);
			float bnoGyro[3];
			BnoDriver.readGyro(bnoGyro);
			float bnoEul[3];
			BnoDriver.readEul(bnoEul);
			addResponse("BNO055 data:\r\n");
			addResponse("x accel: %f\r\n", bnoAccel[0]);
			addResponse("y accel: %f\r\n", bnoAccel[1]);
			addResponse("z accel: %f\r\n", bnoAccel[2]);
			addResponse("x gyro : %f\r\n", bnoGyro[0]);
			addResponse("y gyro : %f\r\n", bnoGyro[1]);
			addResponse("z gyro : %f\r\n", bnoGyro[2]);
			addResponse("x eul  : %f\r\n", bnoEul[0]);
			addResponse("y eul  : %f\r\n", bnoEul[1]);
			addResponse("z eul  : %f\r\n", bnoEul[2]);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "init")) {
			if (isOK(BnoDriver.init())) {
				addResponse("BNO successfully reinitialised\r\n");
				statusPacket.init_status |= 0b00010000;
			} else {
				addResponse("Failed to reinitialise BNO\r\n");
				statusPacket.init_status &= 0b11101111;
			}
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "raw")) {
			int16_t bnoAccel[3];
			BnoDriver.readRawAccel(bnoAccel);
			int16_t bnoGyro[3];
			BnoDriver.readRawGyro(bnoGyro);
			int16_t bnoEul[3];
			BnoDriver.readRawEul(bnoEul);
			addResponse("BNO055 raw data:\r\n");
			addResponse("x accel: %d\r\n", bnoAccel[0]);
			addResponse("y accel: %d\r\n", bnoAccel[1]);
			addResponse("z accel: %d\r\n", bnoAccel[2]);
			addResponse("x gyro : %d\r\n", bnoGyro[0]);
			addResponse("y gyro : %d\r\n", bnoGyro[1]);
			addResponse("z gyro : %d\r\n", bnoGyro[2]);
			addResponse("x eul  : %d\r\n", bnoEul[0]);
			addResponse("y eul  : %d\r\n", bnoEul[1]);
			addResponse("z eul  : %d\r\n", bnoEul[2]);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "accel")) {
			int16_t bnoAccel[3];
			BnoDriver.readRawAccel(bnoAccel);
			int16_t bnoGrav[3];
			BnoDriver.readRawGravity(bnoGrav);
			int16_t bnoLia[3];
			BnoDriver.readRawLia(bnoLia);
			addResponse("BNO055 raw data:\r\n");
			addResponse("x accel: %d\r\n", bnoAccel[0]);
			addResponse("y accel: %d\r\n", bnoAccel[1]);
			addResponse("z accel: %d\r\n", bnoAccel[2]);
			addResponse("x grav : %d\r\n", bnoGrav[0]);
			addResponse("y grav : %d\r\n", bnoGrav[1]);
			addResponse("z grav : %d\r\n", bnoGrav[2]);
			addResponse("x lia  : %d\r\n", bnoLia[0]);
			addResponse("y lia  : %d\r\n", bnoLia[1]);
			addResponse("z lia  : %d\r\n", bnoLia[2]);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "temp")) {
			float temp;
			BnoDriver.readTemp(&temp);
			addResponse("BNO055 temp:\r\n");
			addResponse("temp: %f\r\n", temp);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "whoami")) {
			addResponse("0x%02X -----> BNO055\r\n", BNO055_CHIP_ID);
			uint8_t whoamiResponse = 0;
			BnoDriver.readRegister(BNO055_CHIP_ID, &whoamiResponse);
			addResponse("0x%02X <----- BNO055\r\n", whoamiResponse);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "registers")) {
			int16_t rawAccelData[3];
			int16_t rawGravityData[3];
			int16_t rawGyroData[3];
			int16_t rawEulData[3];
			int16_t rawLiaData[3];
			UFC_ECODE status = BnoDriver.readRawAccel(rawAccelData);
			if (isOK(status)) {
				addResponse("readRawAccel returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawAccel returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("ACC_DATA_X: %d\r\n", rawAccelData[0]);
			addResponse("ACC_DATA_Y: %d\r\n", rawAccelData[1]);
			addResponse("ACC_DATA_Z: %d\r\n", rawAccelData[2]);
			status = BnoDriver.readRawGravity(rawGravityData);
			if (isOK(status)) {
				addResponse("readRawGravity returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawGravity returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("GRV_DATA_X: %d\r\n", rawGravityData[0]);
			addResponse("GRV_DATA_Y: %d\r\n", rawGravityData[1]);
			addResponse("GRV_DATA_Z: %d\r\n", rawGravityData[2]);
			status = BnoDriver.readRawLia(rawLiaData);
			if (isOK(status)) {
				addResponse("readRawGravity returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawGravity returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("LIA_DATA_X: %d\r\n", rawLiaData[0]);
			addResponse("LIA_DATA_Y: %d\r\n", rawLiaData[1]);
			addResponse("LIA_DATA_Z: %d\r\n", rawLiaData[2]);
			status = BnoDriver.readRawGyro(rawGyroData);
			if (isOK(status)) {
				addResponse("readRawGyro returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawGyro returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("GYR_DATA_X: %d\r\n", rawGyroData[0]);
			addResponse("GYR_DATA_Y: %d\r\n", rawGyroData[1]);
			addResponse("GYR_DATA_Z: %d\r\n", rawGyroData[2]);
			status = BnoDriver.readRawEul(rawEulData);
			if (isOK(status)) {
				addResponse("readRawEul returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawEul returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("EUL_HEADING: %d\r\n", rawEulData[0]);
			addResponse("EUL_ROLL   : %d\r\n", rawEulData[1]);
			addResponse("EUL_PITCH  : %d\r\n", rawEulData[2]);
			uint8_t temperature;
			BnoDriver.readRegister(BNO055_TEMP, &temperature);
			addResponse("TEMP: %d\r\n", (int8_t) temperature);
			uint8_t chipidresponse = 0;
			BnoDriver.readRegister(BNO055_CHIP_ID, &chipidresponse);
			addResponse("CHIP_ID: 0x%02X\r\n", chipidresponse);
			uint8_t accidresponse = 0;
			BnoDriver.readRegister(BNO055_ACC_ID, &accidresponse);
			addResponse("ACC_ID: 0x%02X\r\n", accidresponse);
			uint8_t gyridresponse = 0;
			BnoDriver.readRegister(BNO055_GYR_ID, &gyridresponse);
			addResponse("GYR_ID: 0x%02X\r\n", gyridresponse);
			uint8_t magidresponse = 0;
			BnoDriver.readRegister(BNO055_MAG_ID, &magidresponse);
			addResponse("MAG_ID: 0x%02X\r\n", magidresponse);
			int16_t accelOffset[3];
			status = BnoDriver.readRawAccelOffset(accelOffset);
			if (isOK(status)) {
				addResponse("readRawAccelOffset returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawAccelOffset returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("ACC_OFFSET_X: %d\r\n", accelOffset[0]);
			addResponse("ACC_OFFSET_Y: %d\r\n", accelOffset[1]);
			addResponse("ACC_OFFSET_Z: %d\r\n", accelOffset[2]);
			int16_t gyroOffset[3];
			status = BnoDriver.readRawGyroOffset(gyroOffset);
			if (isOK(status)) {
				addResponse("readRawAccelOffset returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawAccelOffset returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("GYR_OFFSET_X: %d\r\n", gyroOffset[0]);
			addResponse("GYR_OFFSET_Y: %d\r\n", gyroOffset[1]);
			addResponse("GYR_OFFSET_Z: %d\r\n", gyroOffset[2]);
			int16_t magOffset[3];
			status = BnoDriver.readRawMagOffset(magOffset);
			if (isOK(status)) {
				addResponse("readRawAccelOffset returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawAccelOffset returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("MAG_OFFSET_X: %d\r\n", magOffset[0]);
			addResponse("MAG_OFFSET_Y: %d\r\n", magOffset[1]);
			addResponse("MAG_OFFSET_Z: %d\r\n", magOffset[2]);
			uint8_t unitresponse = 0;
			BnoDriver.readRegister(BNO055_UNIT_SEL, &unitresponse);
			addResponse("UNIT_SEL: 0x%02X\r\n", unitresponse);
			uint8_t stresult = 0;
			BnoDriver.readRegister(BNO055_ST_RESULT, &stresult);
			addResponse("ST_RESULT: 0x%02X\r\n", stresult);
			uint8_t oprmode = 0;
			BnoDriver.readRegister(BNO055_OPR_MODE, &oprmode);
			addResponse("OPR_MODE: 0x%02X\r\n", oprmode);
			uint8_t calibresponse = 0;
			BnoDriver.readRegister(BNO055_CALIB_STAT, &calibresponse);
			addResponse("CALIB_STAT: 0x%02X\r\n", calibresponse);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "speedtest")) {
			uint32_t initMs = HAL_GetTick();
			float bnoAccel[3];
			float bnoGyro[3];
			float bnoEul[3];

			UFC_ECODE status = UFC_ECODE_OK;
			uint32_t successCount = 0;
			uint32_t failCount = 0;

			while (HAL_GetTick() - initMs < 1000) {
				status = UFC_ECODE_OK;
				status |= BnoDriver.readAccel(bnoAccel);
				status |= BnoDriver.readGyro(bnoGyro);
				status |= BnoDriver.readEul(bnoEul);

				if (status == UFC_ECODE_OK) {
					successCount++;
				} else {
					failCount++;
				}
			}
			addResponse("Total packets received: %d\r\n", successCount + failCount);
			addResponse("Successes: %d\r\n", successCount);
			addResponse("Failures: %d\r\n", failCount);
			addResponse("Success rate: %d\r\n",  successCount * 100 / (failCount + successCount));
			return UFC_ECODE_OK;
		}
	}
	if (streq(argv[0], "alt")) {
		if (argc == 1) {
			float altData[2];
			AltDriver.readData(altData);
			addResponse("BMP390 data:\r\n");
			addResponse("temperature: %lf\r\n", altData[0]);
			addResponse("pressure: %lf\r\n", altData[1]);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "init")) {
			if (isOK(AltDriver.init())) {
				addResponse("BMP390 successfully reinitialised\r\n");
				statusPacket.init_status |= 0b00100000;
			} else {
				addResponse("Failed to reinitialise BMP390\r\n");
				statusPacket.init_status &= 0b11011111;
			}
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "raw")) {
			uint32_t altRawData[2];
			AltDriver.readRawData(altRawData);
			addResponse("BMP390 raw data:\r\n");
			addResponse("temperature: %d\r\n", altRawData[0]);
			addResponse("pressure: %d\r\n", altRawData[1]);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "whoami")) {
			addResponse("0x%02X -----> BMP390\r\n", BMP390_CHIP_ID | 0b10000000);
			uint8_t whoamiResponse = 0;
			AltDriver.readRegister(BMP390_CHIP_ID, &whoamiResponse);
			addResponse("0x%02X <----- BMP390\r\n", whoamiResponse);
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "speedtest")) {
			uint32_t initMs = HAL_GetTick();
			float altData[2];

			UFC_ECODE status = UFC_ECODE_OK;
			uint32_t successCount = 0;
			uint32_t failCount = 0;

			while (HAL_GetTick() - initMs < 1000) {
				status = AltDriver.readData(altData);

				if (status == UFC_ECODE_OK) {
					successCount++;
				} else {
					failCount++;
				}
			}
			addResponse("Total packets received: %d\r\n", successCount + failCount);
			addResponse("Successes: %d\r\n", successCount);
			addResponse("Failures: %d\r\n", failCount);
			addResponse("Success rate: %d\r\n",  successCount * 100 / (failCount + successCount));
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "registers")) {
			uint8_t statusRegBefore;
			AltDriver.readRegister(BMP390_STATUS, &statusRegBefore);
			addResponse("BMP390_STATUS: %d\r\n", statusRegBefore);
			uint32_t rawAltData[2];
			uint8_t rawRegisters[6];
			UFC_ECODE status = AltDriver.readRawData(rawAltData);
			if (isOK(status)) {
				addResponse("readRawData returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("readRawData returned UFC_ECODE_FAIL\r\n");
			}
			addResponse("OUT_TEMPERATURE: %d\r\n", rawAltData[0]);
			addResponse("OUT_PRESSURE: %d\r\n", rawAltData[1]);
			AltDriver.readRegister(BMP390_TEMP_LOW,      &rawRegisters[0]);
			AltDriver.readRegister(BMP390_TEMP_MID,      &rawRegisters[1]);
			AltDriver.readRegister(BMP390_TEMP_HIGH,     &rawRegisters[2]);
			AltDriver.readRegister(BMP390_PRESSURE_LOW,  &rawRegisters[3]);
			AltDriver.readRegister(BMP390_PRESSURE_MID,  &rawRegisters[4]);
			AltDriver.readRegister(BMP390_PRESSURE_HIGH, &rawRegisters[5]);
			addResponse("BMP390_TEMP_LOW: %d\r\n", rawRegisters[0]);
			addResponse("BMP390_TEMP_MID: %d\r\n", rawRegisters[1]);
			addResponse("BMP390_TEMP_HIGH: %d\r\n", rawRegisters[2]);
			addResponse("BMP390_PRESSURE_LOW: %d\r\n", rawRegisters[3]);
			addResponse("BMP390_PRESSURE_MID: %d\r\n", rawRegisters[4]);
			addResponse("BMP390_PRESSURE_HIGH: %d\r\n", rawRegisters[5]);
			uint8_t statusReg;
			AltDriver.readRegister(BMP390_STATUS, &statusReg);
			addResponse("BMP390_STATUS: %d\r\n", statusReg);
			uint8_t configReg;
			AltDriver.readRegister(BMP390_CONFIG, &configReg);
			addResponse("BMP390_CONFIG: %d\r\n", configReg);
			uint8_t ifConfReg;
			AltDriver.readRegister(BMP390_IF_CONF, &ifConfReg);
			addResponse("BMP390_IF_CONF: %d\r\n", ifConfReg);
			uint8_t pwrReg;
			AltDriver.readRegister(BMP390_PWR_CTRL, &pwrReg);
			addResponse("BMP390_PWR_CTRL: %d\r\n", pwrReg);
			uint8_t osrReg;
			AltDriver.readRegister(BMP390_OSR, &osrReg);
			addResponse("BMP390_OSR: %d\r\n", osrReg);
			uint8_t odrReg;
			AltDriver.readRegister(BMP390_ODR, &odrReg);
			addResponse("BMP390_ODR: %d\r\n", odrReg);
			uint8_t errReg;
			AltDriver.readRegister(BMP390_ERR_REG, &errReg);
			addResponse("BMP390_ERR_REG: %d\r\n", errReg);
			uartPrintf("BMP390_PAR_T1: %.20lf\r\n", AltDriver.par_t1);
			uartPrintf("BMP390_PAR_T2: %.20lf\r\n", AltDriver.par_t2);
			uartPrintf("BMP390_PAR_T3: %.20lf\r\n", AltDriver.par_t3);
			uartPrintf("BMP390_PAR_P1: %.20lf\r\n", AltDriver.par_p1);
			uartPrintf("BMP390_PAR_P2: %.20lf\r\n", AltDriver.par_p2);
			uartPrintf("BMP390_PAR_P3: %.20lf\r\n", AltDriver.par_p3);
			uartPrintf("BMP390_PAR_P4: %.20lf\r\n", AltDriver.par_p4);
			uartPrintf("BMP390_PAR_P5: %.20lf\r\n", AltDriver.par_p5);
			uartPrintf("BMP390_PAR_P6: %.20lf\r\n", AltDriver.par_p6);
			uartPrintf("BMP390_PAR_P7: %.20lf\r\n", AltDriver.par_p7);
			uartPrintf("BMP390_PAR_P8: %.20lf\r\n", AltDriver.par_p8);
			uartPrintf("BMP390_PAR_P9: %.20lf\r\n", AltDriver.par_p9);
			uartPrintf("BMP390_PAR_P10: %.20lf\r\n", AltDriver.par_p10);
			uartPrintf("BMP390_PAR_P11: %.20lf\r\n", AltDriver.par_p11);
			return UFC_ECODE_OK;
		}
	}
	if (streq(argv[0], "gps")) {
		if (argc == 1) {
			uint8_t success = 0;
			uint32_t startTick = HAL_GetTick();
			while (HAL_GetTick() - startTick < 120) { // should be minimum 100ms for another packet to be produced
				if (GpsDriver.getBufferLevel() > 0) {
					GpsDriver.getNavSolution(&gpsPacket);
					success = 1;
					break;
				}
			}
			if (success) {
				addResponse("GPS data:\r\n");
				addResponse("Time of Week: %d\r\n", gpsPacket.time_of_week);
				addResponse("Time: %d:%d:%d:%d\r\n", gpsPacket.time_hour, gpsPacket.time_min, gpsPacket.time_sec, gpsPacket.time_nanosec);
				addResponse("Time accuracy: ±%d\r\n", gpsPacket.time_accuracy);
				addResponse("Position: %f degrees, %f degrees\r\n", gpsPacket.pos_lat, gpsPacket.pos_lon);
				addResponse("Height above mean sea level: %d\r\n", gpsPacket.height_msl);
				addResponse("Height above ellipsoid: %d\r\n", gpsPacket.height_elip);
				addResponse("Fix Type: %d\r\n", gpsPacket.fix_type);
				addResponse("Number of Satellites: %d\r\n", gpsPacket.num_satellites);
				addResponse("Vertical Accuracy: ±%d\r\n", gpsPacket.vertical_accuracy);
				addResponse("Horizontal Accuracy: ±%d\r\n", gpsPacket.horizontal_accuracy);
				addResponse("Position DOP: %f\r\n", gpsPacket.p_DOP);
				addResponse("Velocity: North: %d East: %d Down: %d\r\n", gpsPacket.vel_north, gpsPacket.vel_east, gpsPacket.vel_down);
				addResponse("Accuracy: ±%d\r\n", gpsPacket.vel_accuracy);
			} else {
				addResponse("GPS request timed out\r\n");
			}
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "whoami")) {
			UFC_ECODE attempt = GpsDriver.whoami();
			if (isOK(attempt)) {
				addResponse("GPS -> ROM SPG 5.10 (7b202e)\r\n");
			} else {
				addResponse("GPSDriver.whoami returned UFC_ECODE_FAIL\r\n");
			}
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "buffer")) {
			addResponse("GPS Buffer has %d bytes\r\n", GpsDriver.getBufferLevel());
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "speedtest")) {
			uint32_t initMs = HAL_GetTick();

			UFC_ECODE status = UFC_ECODE_OK;
			uint32_t successCount = 0;
			uint32_t failCount = 0;

			while (HAL_GetTick() - initMs < 1000) {
				if (GpsDriver.getBufferLevel() > 0) {
					status = GpsDriver.getNavSolution(&gpsPacket);
					if (status == UFC_ECODE_OK) {
						successCount++;
					} else {
						failCount++;
					}
				}
			}
			addResponse("Total packets received: %d\r\n", successCount + failCount);
			addResponse("Successes: %d\r\n", successCount);
			addResponse("Failures: %d\r\n", failCount);
			addResponse("Success rate: %d\r\n",  successCount * 100 / (failCount + successCount));
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "init")) {
			if (isOK(GpsDriver.init())) {
				addResponse("GPS successfully reinitialised\r\n");
				statusPacket.init_status |= 0b01000000;
			} else {
				addResponse("Failed to reinitialise GPS\r\n");
				statusPacket.init_status &= 0b10111111;
			}
			return UFC_ECODE_OK;
		}
	}
	if (streq(argv[0], "lora")) {
		if (argc == 2 && streq(argv[1], "init")) {
			if (isOK(LoRaDriver.init())) {
				addResponse("LoRa successfully reinitialised\r\n");
				statusPacket.init_status |= 0b10000000;
			} else {
				addResponse("Failed to reinitialise LoRa\r\n");
				statusPacket.init_status &= 0b01111111;
			}
			return UFC_ECODE_OK;
		}
		if (argc == 2 && streq(argv[1], "help")) {
			addResponse("LoRa Commands:\r\n");
			// TODO: add help commands
			return UFC_ECODE_OK;
		}

		// TODO: new tx command

		if (argc > 2 && streq(argv[1], "send")) {
			UFC_ECODE status = UFC_ECODE_OK;
			char buffer[256] = {0};
			for (int i = 2; i < argc - 1; i++) {
				strcat(buffer, argv[i]);
				strcat(buffer, " ");
			}
			strcat(buffer, argv[argc - 1]);
			strcat(buffer, "\r\n");
			status |= LoRaDriver.tx((uint8_t *) buffer, strlen(buffer));
			if (isOK(status)) {
				addResponse("LORA tx returned UFC_ECODE_OK\r\n");
			} else {
				addResponse("LORA tx returned UFC_ECODE_FAIL\r\n");
			}
			return UFC_ECODE_OK;
		}

		// TODO: add command to modify lora settings

	}
	if (argc == 2 && streq(argv[0], "i2c") && streq(argv[1], "restart")) {
		BnoDriver.i2c.init();
		addResponse("attempted reset of I2C peripheral\r\n");
		return UFC_ECODE_OK;
	}
	return UFC_ECODE_FAIL;
}
