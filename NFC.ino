
void nfcInit()						// initialize the CR95HF chip / BM019 module
{
	check_stack("#nfci#");

	configSPI(1);
	NFC_wakeUP(1);
	NFCReady = 0;
	SetNFCprotocolCommand(1);
	runIDNCommand(10);
	idnDataFromIDNResponse();
	printIDNData();
}


void configSPI(int show)					// set SPI pinMode and params
{
	check_stack("#cspi#");

	if (show) print_state("configSPI()");
	pinMode(PIN_SPI_SS, OUTPUT);
	pinMode(PIN_IRQ, OUTPUT);
	SPI.begin();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.setFrequency(1000);
	if (show) Serial.print(" - done");
}

void NFC_wakeUP(int show)			//send wake up pulse to CR95HF and configure to use SPI
{
	check_stack("#nfcWU#");
	if (show) print_state("NFC_wakeUp()");
	digitalWrite(PIN_IRQ, HIGH);
	delay(10);
	digitalWrite(PIN_IRQ, LOW);
	delayMicroseconds(100);
	digitalWrite(PIN_IRQ, HIGH);
	delay(10);
	if (show) Serial.print(" - done");
}

void SetNFCprotocolCommand(int show)
{
	check_stack("#stNFCPtCmd#");
	for (int t = 0; t < 9; t++)
	{
		if (show) print_state("SetNFCprotocolCommand()");
		const int length = 4;
		byte command[length];
		command[0] = 0x02;
		command[1] = 0x02;
		command[2] = 0x01;
		command[3] = 0x0F;
		send_NFC_PollReceive(command, sizeof(command));
		if ((resultBuffer[0] == 0) & (resultBuffer[1] == 0)) 
		{
			if (t > 0) 
			{
				Serial.print(", Try = ");
				Serial.print(t);
			}			
			NFCReady = 1;
			break;
		}
		else 
		{
			Serial.print(" - error, resultBuffer: ");
			for (byte i = 0; i < 2; i++) 
			{
				Serial.print(resultBuffer[i], HEX);
				Serial.print(" ");
			}
			Serial.print(", Try #");
			Serial.print(t);
			Serial.print(" - BAD RESPONSE TO SET PROTOCOL");
			NFCReady = 0; // NFC not ready
		}
	}
	if (show) Serial.print(" - done");
}

void runIDNCommand(int maxTrials)  // requests information about the CR95HF and its revision, command returns the device ID and the ROM CRC 
{
	check_stack("#ridnc#");
	byte command[2];
	print_state("runIDNCommand() - ");
	command[0] = 0x01;
	command[1] = 0x00;
	delay(10);	
	runIDNCommandUntilNoError(command, sizeof(command), maxTrials);
	Serial.print("done");
}

void idnDataFromIDNResponse()
{
	check_stack("#idndfir#");

	idnData.resultCode = resultBuffer[0];
	for (int i = 0; i < 13; i++)
	{
		idnData.deviceID[i] = resultBuffer[i + 2];
	}
	idnData.romCRC[0] = resultBuffer[13 + 2];
	idnData.romCRC[1] = resultBuffer[14 + 2];
	return;
}

void printIDNData()
{
	char nfc[NFCDEVICEID];
	int i;

	check_stack("#prtIDNDta#");

	Serial.printf("\r\n\tNFC Device ID: %x", idnData.deviceID[0]);
	nfc[0] = (char)idnData.deviceID[0];
	for (i = 1; i < 12; i++)
	{
		Serial.printf(":%x", idnData.deviceID[i]);
		nfc[i] = (char)idnData.deviceID[i];
	}
	nfc[i] = '\0';
	Serial.printf(" (%s)", nfc);
	Serial.printf("\r\n\tNFC Device CRC  %x:%x", idnData.romCRC[0], idnData.romCRC[1]);
}

void send_NFC_PollReceive(byte *command, int commandLength)
{	
	check_stack("#sndNFCPllRcv#");
	send_NFC_Command(command, commandLength);
	poll_NFC_UntilResponsIsReady();
	receive_NFC_Response();
}

void runIDNCommandUntilNoError(byte *command, int length, int maxTrials)
{
	int count = 0;
	bool success;
	check_stack("#ridncune#");
	do
	{
		count++;
		memset(resultBuffer, 0, sizeof(resultBuffer));
		send_NFC_PollReceive(command, sizeof(command));
		success = idnResponseHasNoError();
	} while (!success && (count < maxTrials));
	delay(10);
}

void send_NFC_Command(byte *commandArray, int length)
{
	//  check_stack("#nfcCmd#");
	digitalWrite(PIN_SPI_SS, LOW);
	SPI.transfer(0x00);
	for (int i = 0; i < length; i++) 
	{
		SPI.transfer(commandArray[i]);
	}
	digitalWrite(PIN_SPI_SS, HIGH);
	delay(1);
}

void poll_NFC_UntilResponsIsReady()
{
	unsigned long ms = mymillis();
	byte rb;
	//  check_stack("#pllNFCURisR#");
	digitalWrite(PIN_SPI_SS, LOW);
	while ((resultBuffer[0] != 8) && ((mymillis() - ms) < NFCTIMEOUT))
	{
		rb = resultBuffer[0] = SPI.transfer(0x03);
		resultBuffer[0] = resultBuffer[0] & 0x08;
	}
	digitalWrite(PIN_SPI_SS, HIGH);
	delay(1);
	if (mymillis() - ms > NFCTIMEOUT) 
	{
		Serial.print("\r\n *** poll timeout *** -> response ");
		Serial.print(rb);
	}
}
void receive_NFC_Response()
{
	//  check_stack("#recNFCRsp#");
	digitalWrite(PIN_SPI_SS, LOW);
	SPI.transfer(0x02);
	resultBuffer[0] = SPI.transfer(0);
	resultBuffer[1] = SPI.transfer(0);
	for (byte i = 0; i < resultBuffer[1]; i++) resultBuffer[i + 2] = SPI.transfer(0);
	digitalWrite(PIN_SPI_SS, HIGH);
	delay(1);
}

bool idnResponseHasNoError()
{
	check_stack("#idnrhne#");
	if (resultBuffer[0] == 0x00)
	{
		return true;
	}
	return false;
}

void readAllData()
{
	check_stack("#rdAllDta#");

	NFC_wakeUP(0);
	NFCReady = 0;
	SetNFCprotocolCommand(0);
	runSystemInformationCommandUntilNoError(10);
	systemInformationDataFromGetSystemInformationResponse();
	printSystemInformationData();
	memcpy(sensor.uid, systemInformationData.uid, sizeof(sensor.uid));		// get current sensor SN
	
	bool identical = 1;
	for (int i = 0; i < 8; i++)				// check if sensor SN has changed
	{
		if (sensor.uid[i] != sensor.oldUid[i]) {
			print_state("sensor UID missing or changed, ");
			identical = 0;
			break;
		}
	}
	
	if (!identical) // if we have a new or changed sensor UID initialize
	{
		Serial.print("reinitialize sensor data structure");
		initSensor();
	}
	
	memcpy(sensor.oldUid, systemInformationData.uid, sizeof(sensor.uid)); // after reinitialization store the current UID for the next loop 
	

	sensorData.sensorDataOK = readSensorData();

	if (sensorData.sensorDataOK) 
	{
		decodeSensor();
		apply_spike_filter();
		put_reading2queue(mymillis(), sensorData.trend[0], 1);
	}
	else 
	{
		print_state("no sensor data received, put 0 to queue");
		put_reading2queue(mymillis(), 0, 1);
	}
	sendNFC_ToHibernate();
}

void runSystemInformationCommandUntilNoError(int maxTrials)
{
	check_stack("#rSysInfCmd#");

	memset(resultBuffer, 0, sizeof(resultBuffer));
	byte command[4];
	command[0] = 0x04;
	command[1] = 0x02;
	command[2] = 0x03;
	command[3] = 0x2B;
	delay(10);
	runNFCcommandUntilNoError(command, sizeof(command), maxTrials);
}

void systemInformationDataFromGetSystemInformationResponse()
{
	check_stack("#sysInfDta#");	
	systemInformationData.resultCode = resultBuffer[0];
	systemInformationData.responseFlags = resultBuffer[2];
	if (systemInformationData.resultCode == 0x80)
	{
		if ((systemInformationData.responseFlags & 0x01) == 0)
		{
			systemInformationData.infoFlags = resultBuffer[3];
			for (int i = 0; i < 8; i++)
			{
				systemInformationData.uid[i] = resultBuffer[11 - i];
			}
			systemInformationData.errorCode = resultBuffer[resultBuffer[1] + 2 - 1];
		}
		else
		{
			systemInformationData.errorCode = resultBuffer[3];
		}
		decodeSN(systemInformationData.uid);
		systemInformationData.sensorSN = decodedSensorSN;
		sensorData.sensorSN = decodedSensorSN;
	}
	else
	{
		print_state(" *** error reading patch info, clearing UID");
		clearBuffer(systemInformationData.uid);
		systemInformationData.errorCode = resultBuffer[3];
	}
	return;
}

void printSystemInformationData()
{
	check_stack("#psid#");  
	print_state("Sensor SN: ");
	Serial.print(systemInformationData.sensorSN);
}

bool readSensorData()
{
	byte resultCode = 0;
	int trials = 0;
	int maxTrials = 10;
	bool crcResult = false;

	check_stack("#rsdta#");

	#ifdef DEBUG
		print_state("read Libre sensor using shadow FRAM mechanism");
	#endif DEBUG

	readHeader();
	if (!checkCRC16(sensor.fram, (byte)0)) readHeader(); // one more time

	readBody();
	if (!checkCRC16(sensor.fram, (byte)1)) readBody(); // one more time

												 
	readFooter(); // @UPetersen: footer will not be used, shall only be read once at program start
	if (!checkCRC16(sensor.fram, (byte)2)) readFooter(); // one more time

	reReadBlocksWhereResultCodeStillHasEnError(); // // try again for blocks that had read errors

		
	clearBuffer(dataBuffer); // to be optimized - copy FRAM sensor data to thr original used array
	for (int i = 0; i < 43 * 8; i++) {
		dataBuffer[i] = sensor.fram[i];
	}
	bool resultH = checkCRC16(dataBuffer, 0);
	bool resultB = checkCRC16(dataBuffer, 1);
	bool resultF = checkCRC16(dataBuffer, 2);

	if (resultH && resultB && resultF)
		crcResult = true;
	else 
	{
		crcResult = false;
		Serial.printf(" - failed, CRC check (Header-Body-Footer) = %d %d %d", resultH, resultB, resultF);
	}

	if (crcResult) NFCReady = 2;
	else NFCReady = 1;

	return crcResult;
}

void sendNFC_ToHibernate()
{	
	const int length = 16;
	byte command[length];
	check_stack("#snfcthib#");
	command[0] = 0x07;
	command[1] = 0x0E;
	command[2] = 0x08;
	command[3] = 0x04;
	command[4] = 0x00;
	command[5] = 0x04;
	command[6] = 0x00;
	command[7] = 0x18;
	command[8] = 0x00;
	command[9] = 0x00;
	command[10] = 0x00;
	command[11] = 0x00;
	command[12] = 0x00;
	command[13] = 0x00;
	command[14] = 0x00;
	command[15] = 0x00;
	send_NFC_Command(command, sizeof(command));
}

void runNFCcommandUntilNoError(byte *command, int length, int maxTrials)
{
	check_stack("#rNFCCmdUNErr#");
	int count = 0;
	bool success;
	do
	{
		delay(1);
		count++;
		send_NFC_PollReceive(command, sizeof(command));
		success = responseHasNoError();
	} while (!success && (count < maxTrials));
	delay(1);
}

bool responseHasNoError()
{
	//  check_stack("#rspHsNoErr#");
	if (resultBuffer[0] == 0x80)
	{
		if ((resultBuffer[2] & 0x01) == 0)
		{
			return true;
		}
	}
	return false;
}

void clearBuffer(byte *tmpBuffer)
{
	check_stack("#clrb#");
	memset(tmpBuffer, 0, sizeof(tmpBuffer));
}

void readHeader() 
{

	int maxTrials = 10; // (Re)read a block max four times
	check_stack("#rh#");
	#ifdef DEBUG
		print_state("read header");
	#endif	
	for (byte block = 0; block < 3; block++) // read first three blocks (0 , 1, 2)
	{
		sensor.resultCodes[block] = readSingleBlock((byte)block, maxTrials);
	#ifdef DEBUG
			Serial.printf("resultCode 0x%x", sensor.resultCodes[block]);
	#endif
	}
	#ifdef DEBUG
		
		if (!checkCRC16(sensor.fram, (byte)0)) // Check crc16 for header
		{
			print_state(" ****************** CRC16 check for header failed");
		}
		else 
		{
			print_state(" CRC16 check for header succeded");
		}
	#endif
}

void readBody() // read complete body data
{
	check_stack("#rb#");

	int maxTrials = 4; // (Re)read a block max four times
	#ifdef DEBUG
		print_state("read body");
	#endif
	
	sensor.resultCodes[39] = readSingleBlock((byte)39, maxTrials); // read block 0x27 (i.e. 39) with minute counter and block 3 with indices on trend and history data
	#ifdef DEBUG
		Serial.printf("resultCode 0x%x", sensor.resultCodes[39]);
	#endif
	sensor.resultCodes[3] = readSingleBlock((byte)3, maxTrials);
	#ifdef DEBUG
		Serial.printf("resultCode 0x%x", sensor.resultCodes[39]);
	#endif
	
	if (sensor.resultCodes[3] == 0x80 && sensor.resultCodes[39] == 0x80) // if no read erros continue with checking for new data
	{
		sensor.oldNextTrend = sensor.nextTrend;
		sensor.nextTrend = sensor.fram[26];
		sensor.oldNextHistory = sensor.nextHistory;
		sensor.nextHistory = sensor.fram[27];
		#ifdef DEBUG
			Serial.printf("resultCode Block 3: 0x%x\n\r", sensor.resultCodes[3]);
			Serial.printf("oldNextTrend:       0x%x\n\r", sensor.oldNextTrend);
			Serial.printf("nextTrend:          0x%x\n\r", sensor.nextTrend);
			Serial.printf("oldNextHistory:     0x%x\n\r", sensor.oldNextHistory);
			Serial.printf("nextHistory:        0x%x\n\r", sensor.nextHistory);
		#endif
		sensor.oldMinutesSinceStart = sensor.minutesSinceStart;
		sensor.minutesSinceStart = ((uint16_t)sensor.fram[317] << 8) + (uint16_t)sensor.fram[316]; // bytes swapped
		uint16_t minutesSinceLastReading = sensor.minutesSinceStart - sensor.oldMinutesSinceStart;
		#ifdef DEBUG
			Serial.printf("resultCode Block 39:     0x%x\n\r", sensor.resultCodes[39]);
			Serial.printf("oldMinutesSinceStart:    0x%x\n\r", sensor.oldMinutesSinceStart);
			Serial.printf("minutesSinceStart:       0x%x\n\r", sensor.minutesSinceStart);
			Serial.printf("minutesSinceLastReading: 0x%x\n\r", minutesSinceLastReading);
		#endif
		
		byte trendDelta = sensor.nextTrend - sensor.oldNextTrend; // amount of new trend data
		byte trendsSinceLastReading = (trendDelta >= 0) ? trendDelta : trendDelta + 16; // compensate ring buffer
		if (minutesSinceLastReading > 15) trendsSinceLastReading = 16;  // Read all 16 trend data if more than 15 minute have passed

		#ifdef DEBUG
			Serial.printf("trendDelta:             0x%x\n\r", trendDelta);
			Serial.printf("trendsSinceLastReading: 0x%x\n\r", trendsSinceLastReading);
		#endif
		
		if (trendsSinceLastReading > 0) // if there is new trend data, read the new trend data, along the ring buffer from old to new
		{
			int firstTrend = sensor.oldNextTrend;
			int firstTrendByte = 28 + (6 * firstTrend);
			int lastTrendByte = firstTrendByte + (6 * trendsSinceLastReading) - 1; // Does not consider the ring buffer (this is considered later in the reading loop)

			byte firstTrendBlock = firstTrendByte / 8; // integer division without remainder
			byte lastTrendBlock = lastTrendByte / 8;   // integer division without remainder

			#ifdef DEBUG
				Serial.printf("firstTrend:      0x%x\n\r", firstTrend);
				Serial.printf("firstTrendByte:  0x%x\n\r", firstTrendByte);
				Serial.printf("lastTrendByte:   0x%x\n\r", lastTrendByte);
				Serial.printf("firstTrendBlock: 0x%x\n\r", firstTrendBlock);
				Serial.printf("lastTrendBlock:  0x%x\n\r", lastTrendBlock);
			#endif
			
			byte trendBlock = firstTrendBlock; // read the trend blocks that are new since last reading, but no more than 100 times to avoid endless loop in error case
			byte count = 0;
			while (count <= 100) 
			{
				if (trendBlock != 3) // block 3 was read already and can be skipped
				{ 
					sensor.resultCodes[trendBlock] = readSingleBlock(trendBlock, maxTrials);
					#ifdef DEBUG
						Serial.printf("resultCode Block 0x%x: 0x%x", trendBlock, sensor.resultCodes[trendBlock]);
					#endif
				}
				if (trendBlock == lastTrendBlock) break;
				trendBlock++;
				if (trendBlock > 15) trendBlock = trendBlock - 13; // block 16 -> start over at block 3
				count++;
			}
		}
	
		byte historyDelta = sensor.nextHistory - sensor.oldNextHistory;  // amount of new history data
		byte historiesSinceLastReading = (historyDelta >= 0) ? historyDelta : historyDelta + 32; // compensate ring buffer
																								 
		if (minutesSinceLastReading > 31 * 15) historiesSinceLastReading = 32; // new: ensure to read all history blocks after program start

		#ifdef DEBUG
			Serial.printf("\r\nhistoryDelta:              0x%x\n\r", historyDelta);
			Serial.printf("historiesSinceLastReading: 0x%x\n\r", historiesSinceLastReading);
		#endif
		
		if (historiesSinceLastReading > 0) // if there is new trend data, read the new trend data
		{
			int firstHistory = sensor.oldNextHistory;
			int firstHistoryByte = 124 + (6 * firstHistory);
			int lastHistoryByte = firstHistoryByte + (6 * historiesSinceLastReading) - 1; // Does not consider the ring buffer (this is considered later in the reading loop)

			byte firstHistoryBlock = firstHistoryByte / 8; // integer division without remainder
			byte lastHistoryBlock = lastHistoryByte / 8;   // integer division without remainder

			#ifdef DEBUG
				Serial.printf("firstHistory:       0x%x\n\r", firstHistory);
				Serial.printf("firstHistoryByte:   0x%x\n\r", firstHistoryByte);
				Serial.printf("lastHistoryByte:    0x%x\n\r", lastHistoryByte);
				Serial.printf("firstHistoryBlock:  0x%x\n\r", firstHistoryBlock);
				Serial.printf("lastHistoryBlock:   0x%x\n\r", lastHistoryBlock);
			#endif
			
			byte historyBlock = firstHistoryBlock; // read the history blocks that are new since last reading, but no more than 100 times
			byte count = 0;
			while (count <= 100) 
			{
				if (historyBlock != 39) // block 39 was read already and can be skipped
				{ 
					sensor.resultCodes[historyBlock] = readSingleBlock(historyBlock, maxTrials);
				}
				if (historyBlock == lastHistoryBlock) break;
				historyBlock++;
				if (historyBlock > 39) historyBlock = historyBlock - 25; // block 40 -> start over at block 15
				count++;
			}
		}

		#ifdef DEBUG		// Check crc16		
			if (!checkCRC16(sensor.fram, (byte)1)) print_state(" ********************** CRC16 check for body failed");
			else print_state(" CRC16 check for body succeded");
		#endif
	}
}

void readFooter() 
{
	check_stack("#rf#");

	int maxTrials = 10; // (Re)read a block max four times
	#ifdef DEBUG
		print_state("read footer");
	#endif
	// read three blocks of footer (40, 41, 42)
	for (byte block = 40; block < 43; block++) 
	{
		sensor.resultCodes[block] = readSingleBlock((byte)block, maxTrials);
		#ifdef DEBUG
			Serial.printf("resultCode 0x%x", sensor.resultCodes[block]);
		#endif
	}

	#ifdef DEBUG		// Check crc16 for header	
		if (!checkCRC16(sensor.fram, (byte)2)) print_state(" ************************* CRC16 check for footer failed");
		else print_state(" CRC16 check for footer succeded");
	#endif
}

void reReadBlocksWhereResultCodeStillHasEnError() 
{
	int maxTrials = 3;
	check_stack("#rrbwrcshee#");

	for (byte block = 0; block < 40; block++) 
	{
		if (sensor.resultCodes[block] != 0x80) 
		{
			sensor.resultCodes[block] = readSingleBlock(block, maxTrials);
			#ifdef DEBUG
				Serial.printf("Reread block #0x%x with result code 0x%x", block, sensor.resultCodes[block]);
			#endif
		}
	}
}

// Reads a (single) block of 8 bytes at the position blockNum from the FreeStyle Libre FRAM and copies
// the content to the corresponding position of the dataBuffer.
// If an error occurs (result code != 128), the reading process is repeated up to maxTrials times.
// The final result code is returned.
// blockNum    number of the block of the FreeStyle Libre FRAM to be read, possible values are 0 to 243.
// maxTrials   maximum number of reading attempts.
// RXBuffer    buffer used for sending and receiving data.
// fram        buffer of bytes the final block of data is copied to at the position corresponding to blockNum.
//             Example 1: If blockNumber is 0, the 8 bytes of data are copied into fram[0] to fram[7] (i.e. 0*8 to 0*8+7)
//             Example 2: If blockNumber is 17, the 8 bytes of data are copied into fram[136] to fram[143] (i.e. 17*8 = 136.. 17*8+7 = 143)
byte readSingleBlock(byte blockNum, int maxTrials) 
{
	byte command[5];

	check_stack("#rsb#");

	command[0] = SENDRECEIVE;   // command code for send receive CR95HF command
	command[1] = 0x03;          // length of data that follows (3 bytes)
								// command[ 2] = 0x02;          // request Flags byte, single carrier, high data rate
	command[2] = 0x03;          // request Flags byte dual carrier, high data rate
	command[3] = 0x20;          // read single block Command for ISO/IEC 15693
	command[4] = blockNum;      // Block number

	int trials = 0;
	do {
		sendPollReceiveSPINew(PIN_SPI_SS, command, sizeof(command), RXBuffer);
		trials++;
		delay(1);

		#ifdef DEBUG // Print to Serial for debugging			
			if (RXBuffer[0] == 0x80) 
			{
				print_state("");
				Serial.printf("readSingleBlock: block #%d:", blockNum);
				
					for (byte i = 3; i < RXBuffer[1] + 3 - 4; i++) 
					{
						Serial.print(RXBuffer[i], HEX);
						Serial.print(" ");
					}				
			}
			else 
			{
				print_state("");
				Serial.printf("readSingleBlock: block #%d not available, response code is: ", blockNum);
				Serial.println(RXBuffer[0], HEX);
			}		
		#endif

	} while (RXBuffer[0] != 0x80 && trials <= maxTrials); // repeat until result code == 0x80 (no error), but only maxTrials times

	if (RXBuffer[0] == 0x80) 
	{
		for (byte i = 0; i < 8; i++) {
			sensor.fram[blockNum * 8 + i] = RXBuffer[i + 3];  // succes case: copy received data to fram
		}
	}
	else 
	{
		for (byte i = 0; i < 8; i++) {
			sensor.fram[blockNum * 8 + i] = 0;  // error case: fill fram with zeros
		}
	}
	return RXBuffer[0];  // result code of command response
}

void sendPollReceiveSPINew(int ssPIN, byte *command, int commandLength, byte *RXBuffer) 
{
	check_stack("#sprspinw#");
	
	sendSPICommand(ssPIN, command, commandLength);	// step 1 send the command, which is stored in RXBuffer	
	pollSPIUntilResponsIsReady(ssPIN, RXBuffer);	// step 2, poll for data ready	
	receiveSPIResponse(ssPIN, RXBuffer); // step 3, read the data into RXBuffer
}

void sendSPICommand(int ssPin, byte *commandArray, int length) 
{
	check_stack("#sspic#");
	digitalWrite(ssPin, LOW);
	SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
	for (int i = 0; i<length; i++) 
	{
		SPI.transfer(commandArray[i]);
	}
	digitalWrite(ssPin, HIGH);
	delay(1);
}

void pollSPIUntilResponsIsReady(int ssPin, byte *RXBuffer) 
{
	check_stack("#pspiurir#");
	digitalWrite(ssPin, LOW);
	while (RXBuffer[0] != 8) {
		RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
		RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set in response byte
	}
	digitalWrite(ssPin, HIGH);
	delay(1);
}

void receiveSPIResponse(int ssPin, byte *RXBuffer) 
{
	check_stack("#rspir#");
	digitalWrite(ssPin, LOW);
	SPI.transfer(0x02);   // SPI control byte for read
	RXBuffer[0] = SPI.transfer(0);  // response code
	RXBuffer[1] = SPI.transfer(0);  // length of data
	for (byte i = 0; i<RXBuffer[1]; i++) RXBuffer[i + 2] = SPI.transfer(0);  // data
	digitalWrite(ssPin, HIGH);
	delay(1);
}
